/*
 * Goodix TLS driver for libfprint -- chip 27c6:5F10 (Milan/ST411SEC family).
 *
 * Copyright (C) 2026 libfprint contributors
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

// Plain FpDevice driver (NOT FpImageDevice): the small 56x176 sensor does not yield
// enough reproducible NBIS minutiae for cross-touch matching, so we match with the
// sigfm (FAST-9 + BRIEF-256 + RANSAC) algorithm, like the equally small Goodix HTK32.
// The FpDevice + host-side sigfm + raw-frame-gallery design follows the
// AndyHazz/goodix53x5 driver (https://github.com/AndyHazz/goodix53x5-libfprint).
//
// Architecture:
//   * Transport is reused verbatim from the goodixtls base (goodix.c / goodixtls.c):
//     activation, TLS-PSK handshake (OpenSSL) with the per-device session key,
//     MCU_GET_IMAGE.
//   * The FpImageDevice scan layer is NOT used; this driver runs its own
//     activate / capture / enroll / verify state machines and feeds each captured
//     8-bit frame to sigfm.
//
// 5F10-specific empirical facts:
//   * ENABLE_CHIP body is "01 02" (the generic helper emits "01 00").
//   * RESET reply number is 1024; firmware string is "GF_ST411SEC_APP_12705";
//     preset-PSK flags are 0xbb020003; MCU config payload is 224 bytes.
//   * Sensor streams 9856 px decoded row-major as 176 scanlines x 56 px, presented
//     STRAIGHT (no transpose) with ridges inverted dark (see process_to_8bit).
//
// Per-device secrets (TLS-PSK session key + 32-byte preset-PSK identity hash)
// are loaded at activation from disk; see load_secrets() and the file paths in
// goodix5f10.h. They are device-specific and not compiled in.

#include "fp-device.h"
#include "fpi-device.h"
#include "fpi-ssm.h"
#include "gusb/gusb-device.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <glib.h>

#include <openssl/ssl.h> // goodixtls.h references SSL/SSL_CTX types

#include "drivers_api.h"
#include "goodix.h"
#include "goodix_proto.h"
#include "goodixtls.h"
#include "goodix5f10.h"
#include "sigfm.h"

#define FP_COMPONENT "goodixtls5f10"

// Sensor geometry. RAW frame on the wire = 8 + (56*176)/4*6 + 5 = 14797 bytes.
#define GOODIX5F10_WIDTH 56
#define GOODIX5F10_HEIGHT 176
#define GOODIX5F10_SCAN_WIDTH 56
#define GOODIX5F10_SCAN_HEIGHT 176
#define GOODIX5F10_PIXELS (GOODIX5F10_WIDTH * GOODIX5F10_HEIGHT)
#define GOODIX5F10_RAW_FRAME_SIZE \
  (8 + (GOODIX5F10_SCAN_HEIGHT * GOODIX5F10_SCAN_WIDTH) / 4 * 6 + 5)

// Enrollment follows the industry recipe for small patch sensors (Apple Touch ID, the
// libfprint MOC drivers, the partial-fingerprint patent literature): collect a FIXED number
// of quality samples into a multi-template gallery, where successive touches are meant to
// OVERLAP so the gallery stays connected. We deliberately do NOT drop "already covered"
// frames -- overlap is the glue that lets a verify touch landing between samples still match;
// dropping it builds a disjoint gallery of islands (a coverage-guided dedup would do
// exactly that, hurting FRR). Good coverage relies on the user varying finger
// position across touches: the enroll API carries only a stage number plus the fixed
// FP_DEVICE_RETRY_* codes, so there is NO channel to instruct "center" vs "edges" -- the UI
// (fprintd/GNOME) only shows generic "reposition slightly" guidance. The driver just banks
// one best-quality frame per touch until it has ENROLL_SAMPLES of them.
//
// ENROLL_SAMPLES (== nr_enroll_stages) is chosen by analogy to the narrow MOC drivers
// (fpcmoc 25, elanmoc up to 17, synaptics/goodixmoc 8): 24 = 8 center + 16 edge touches.
// FAR/FRR on this sensor are not yet characterised, so treat the count as a starting point.
#define GOODIX5F10_ENROLL_SAMPLES 24  // gallery frames to collect (== nr_enroll_stages)
#define GOODIX5F10_ENROLL_IGNORE_LIMIT 30  // consecutive low-quality touches before giving up
// Minimum SIFT keypoints for an enroll frame to enter the gallery. Real ridge patches give
// ~260 keypoints; an empty/noise frame gives very few. Set well below the real-frame count so
// only degenerate frames are dropped (env-overridable for tuning).
#define GOODIX5F10_ENROLL_MIN_KP  12

// Multi-frame capture per single finger press. The sensor patch is tiny (~5x4 mm), so a
// single touch covers only a fraction of the finger pad and a verify touch that lands on an
// uncovered zone misses entirely (the dominant FRR cause: misses score 0, not "weak"). A held
// finger micro-shifts over ~1 s, so grabbing several back-to-back MCU_GET_IMAGE frames per
// press yields slightly different patches. On verify this multiplies the chance one frame
// overlaps the gallery (P(miss) ~ p^N); on enroll it widens gallery coverage. GET_IMAGE
// rescans the sensor each call, so consecutive frames are genuinely fresh (no re-arm needed).
#define GOODIX5F10_VERIFY_FRAMES 6
#define GOODIX5F10_ENROLL_FRAMES 3

// sigfm match gates:
//   THRESHOLD  -- a per-sample sigfm score at/above this counts the sample as matching.
//   BEST_MIN   -- the best single-sample score must reach this for an overall match.
//   MIN_SAMPLES-- at least this many gallery samples must individually pass THRESHOLD.
//
// sigfm score gates for the buxel pure-C matcher (FAST-9 + BRIEF-256 + RANSAC).
// MIN_SAMPLES is kept at 1: a high best-sample score already implies enough matching
// samples, so the match is gated on BEST_MIN. These are starting values; FAR/FRR on
// this sensor are not yet characterised.
#define GOODIX5F10_SIGFM_THRESHOLD   10
#define GOODIX5F10_SIGFM_BEST_MIN    10
#define GOODIX5F10_SIGFM_MIN_SAMPLES 1

/* Per-frame keypoint cap passed to sigfm_extract_ex. Our 56x176 frames
 * saturate the upstream-default 128 cap (natural ceiling ~208 keypoints
 * per frame). 192 is used as a starting point; higher values showed no
 * apparent improvement. */
#define GOODIX5F10_SIGFM_MAX_KP      192

// ---- FDT (finger detection) ------------------------------------------------------------
// The chip runs GOODIX5F10_FDT_CHANNELS capacitive detection channels. Every FDT command
// answers with [irq u16][touch flag u16][one u16 reading per channel], and takes a
// threshold table in the same channel order.
#define GOODIX5F10_FDT_CHANNELS    10
#define GOODIX5F10_FDT_REPLY_LEN   (4 + 2 * GOODIX5F10_FDT_CHANNELS)
#define GOODIX5F10_FDT_PAYLOAD_LEN 34  // 2 header bytes, 10 threshold pairs, tail padding
#define GOODIX5F10_FDT_THRESH_OFF  2
// Offset added over the idle level when building a threshold, in the chip's half-scale
// units. This is not the detection margin - the chip decides for itself, and it takes
// roughly 25 below the threshold to fire: the deepest reading it left unflagged here was
// 20 under, the shallowest it flagged was 28 under. Idle drift of a few units sits well
// inside that, which is why the table can be rebuilt from any resting reading without the
// wobble mattering. 2 is what the vendor stack used, and it reproduces fdt_reference_
// thresholds below exactly.
#define GOODIX5F10_FDT_DELTA       2

// A baseline that comes back empty makes the subtraction below a no-op, and the touch ends
// up preprocessed unlike every other one. Healthy frames here sit at 2420 of the 12-bit
// range; the 550c driver saw dead ones averaging 0 to 10 (see the commit for their
// write-up). 256 rejects those without judging exposure.
#define GOODIX5F10_BASELINE_MIN_MEAN 256
#define GOODIX5F10_BASELINE_RETRIES  8
#define GOODIX5F10_BASELINE_RETRY_MS 120

typedef guint16 Goodix5f10Pix;

struct _FpiDeviceGoodixTls5f10
{
  FpiDeviceGoodixTls parent;

  FpiSsm       *task_ssm;

  Goodix5f10Pix *calibration_img; // baseline (empty-sensor) frame, scan_w*scan_h
  GPtrArray     *capture_frames;  // 8-bit frames (guint8*, WIDTH*HEIGHT) from the current press
  guint          frames_target;   // how many frames to grab per press (enroll vs verify)
  gboolean       rearm_between;    // GOODIX5F10_REARM: send NAV0 between burst frames (fallback)
  guint          baseline_retries; // consecutive no-signal baseline frames this press

  GPtrArray *enroll_images;       // array of guint8* (8-bit frames) kept in the gallery

  // fixed-count enrollment bookkeeping
  guint      enroll_touches;      // total touches taken this enroll
  guint      enroll_ignored;      // consecutive low-quality touches (no usable frames)
  guint      enroll_target;       // gallery frames to collect (env-overridable)
  guint      enroll_min_kp;       // min SIFT keypoints to accept a frame

  // per-device factory calibration parsed from OTP (0xa6). Each calibration field
  // is stored with triple redundancy (value, complement, duplicate); see parse_otp.
  guint8     config[sizeof (goodix_5f10_config)]; // config as sent, DAC rewritten per device
  guint8     otp_dac_h;           // factory DAC_H (OTP[0x1F]); 0 if redundancy failed
  guint8     otp_tcode;           // factory TCODE (OTP[0x16]); 0 if redundancy failed
  gboolean   otp_dac_valid;
  gboolean   otp_tcode_valid;

  // FDT threshold table sent with every FDT command, in wire form. Seeded from the
  // reference table and replaced by one measured on this sensor; see update_fdt_base.
  guint8     fdt_base[GOODIX5F10_FDT_PAYLOAD_LEN];
  gboolean   fdt_base_live;       // TRUE once the table came from a live reading

  // Per-device TLS session key loaded from disk at activation.
  guint8     tls_psk[GOODIX_5F10_PSK_LEN];
};

G_DECLARE_FINAL_TYPE (FpiDeviceGoodixTls5f10, fpi_device_goodixtls5f10, FPI,
                      DEVICE_GOODIXTLS5F10, FpiDeviceGoodixTls);
G_DEFINE_TYPE (FpiDeviceGoodixTls5f10, fpi_device_goodixtls5f10,
               FPI_TYPE_DEVICE_GOODIXTLS);

// FDT threshold table captured on the development unit, kept only as the seed for the
// first FDT command of a session. Every pair is that sensor's idle reading halved plus
// GOODIX5F10_FDT_DELTA, duplicated across both bytes -- the packing the published 53x5
// notes write as (v & 0xfffe) * 0x80 | v >> 1. Idle levels differ from sensor to sensor,
// so this table describes one laptop and nothing else; update_fdt_base measures the real
// one on first use.
static const guint8 fdt_reference_thresholds[] = {
  0x09, 0x01, 0xaf, 0xaf, 0xb1, 0xb1, 0xaf, 0xaf, 0xb3, 0xb3, 0xae, 0xae,
  0xad, 0xad, 0xb4, 0xb4, 0xae, 0xae, 0xb1, 0xb1, 0xb3, 0xb3, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
G_STATIC_ASSERT (sizeof (fdt_reference_thresholds) == GOODIX5F10_FDT_PAYLOAD_LEN);

// ---- frame decode / processing (copied from the goodix5xx base; small + self-contained)

static void
decode_frame (Goodix5f10Pix *frame, guint32 frame_size, const guint8 *raw_frame)
{
  Goodix5f10Pix *pix = frame;

  for (guint32 i = 8; i != frame_size - 5; i += 6)
    {
      const guint8 *chunk = raw_frame + i;
      *pix++ = ((chunk[0] & 0xf) << 8) + chunk[1];
      *pix++ = (chunk[3] << 4) + (chunk[0] >> 4);
      *pix++ = ((chunk[5] & 0xf) << 8) + chunk[2];
      *pix++ = (chunk[4] << 4) + (chunk[5] >> 4);
    }
}

// Ridge pixels read LOWER than the empty-sensor baseline, so (baseline - finger) is the
// positive ridge signal; clamp at 0.
static void
linear_subtract_inplace (Goodix5f10Pix *src, const Goodix5f10Pix *by, guint len)
{
  for (guint n = 0; n != len; ++n)
    src[n] = (by[n] > src[n]) ? (by[n] - src[n]) : 0;
}

static void
squash_frame_linear (const Goodix5f10Pix *frame, guint8 *squashed, guint len)
{
  Goodix5f10Pix min = 0xffff, max = 0;

  for (guint i = 0; i != len; ++i)
    {
      const Goodix5f10Pix p = frame[i];
      if (p < min) min = p;
      if (p > max) max = p;
    }
  for (guint i = 0; i != len; ++i)
    {
      const Goodix5f10Pix p = frame[i];
      squashed[i] = (max - min == 0) ? 0 : (p - min) * 0xff / (max - min);
    }
}

// The decoded buffer is already in row-major order for a HEIGHT x WIDTH (176 x 56) image:
// consecutive WIDTH(56) decoded pixels form one scanline. Read it straight (NOT transposed)
// and invert so ridges are DARK. The previous transpose (col*HEIGHT+row) SCRAMBLED the image
// into a herringbone moire. The straight reshape(176,56) yields clean curved ridges;
// the transpose yields garbage.
// Returns a newly g_malloc'd WIDTH*HEIGHT 8-bit buffer (caller owns).
static guint8 *
process_to_8bit (const guint8 *squashed_native)
{
  guint8 *img = g_malloc (GOODIX5F10_PIXELS);

  for (int row = 0; row != GOODIX5F10_HEIGHT; ++row)
    for (int col = 0; col != GOODIX5F10_WIDTH; ++col)
      img[col + row * GOODIX5F10_WIDTH] =
        255 - squashed_native[col + row * GOODIX5F10_WIDTH];

  const char *dump = g_getenv ("GOODIX5F10_DUMP");
  if (dump)
    {
      static unsigned seq = 0;
      char path[512];
      g_snprintf (path, sizeof (path), "%s%03u.pgm", dump, seq++);
      FILE *f = fopen (path, "wb");
      if (f)
        {
          fprintf (f, "P5\n%d %d\n255\n", GOODIX5F10_WIDTH, GOODIX5F10_HEIGHT);
          fwrite (img, 1, GOODIX5F10_PIXELS, f);
          fclose (f);
        }
    }
  return img;
}

// ---- generic SSM check callbacks ------------------------------------------------------

static void
check_none (FpDevice *dev, gpointer ssm, GError *error)
{
  if (error) { fpi_ssm_mark_failed (ssm, error); return; }
  fpi_ssm_next_state (ssm);
}

static void
check_none_cmd (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  if (err) { fpi_ssm_mark_failed (ssm, err); return; }
  fpi_ssm_next_state (ssm);
}

// ---- FDT (finger detection) thresholds -------------------------------------------------

static void
dump_fdt_reply (const char *tag, guint8 *data, guint16 len)
{
  g_autofree gchar *hex = data_to_str (data, len);
  g_autoptr (GString) chans = g_string_new (NULL);

  fp_dbg ("FDT %s reply: %u bytes: %s", tag, len, hex);
  if (len != GOODIX5F10_FDT_REPLY_LEN)
    return;

  for (guint i = 0; i < GOODIX5F10_FDT_CHANNELS; i++)
    g_string_append_printf (chans, " %u",
                            (guint) (data[4 + 2 * i] | (data[5 + 2 * i] << 8)));
  fp_dbg ("FDT %s: irq 0x%04x, touch flag 0x%04x, channels:%s", tag,
          (guint) (data[0] | (data[1] << 8)), (guint) (data[2] | (data[3] << 8)),
          chans->str);
}

// Rebuild the FDT threshold table from a live idle reading.
//
// The table is per-unit: each channel's threshold is that sensor's resting level halved,
// plus a small margin. Two sensors of the same model rest at different levels, so a table
// captured on one of them puts another one's resting state on the wrong side of the
// comparison -- FDT-down then returns without a finger and the driver captures air
// (github issue #2). Measure it on the device in front of us instead.
static void
update_fdt_base (FpiDeviceGoodixTls5f10 *self, const char *tag,
                 guint8 *data, guint16 len)
{
  const gboolean bootstrap = !self->fdt_base_live;
  guint16 touch_flag;

  if (len != GOODIX5F10_FDT_REPLY_LEN)
    {
      fp_warn ("FDT %s: reply is %u bytes, expected %u; keeping the current thresholds",
               tag, len, (guint) GOODIX5F10_FDT_REPLY_LEN);
      return;
    }

  // A reading taken with a finger on the sensor is not a resting level. The first reading
  // of a session is taken regardless: until it lands, the thresholds belong to whichever
  // unit the reference table was captured on, and anything measured here beats that.
  touch_flag = data[2] | ((guint16) data[3] << 8);
  if (touch_flag != 0 && !bootstrap)
    return;

  for (guint i = 0; i < GOODIX5F10_FDT_CHANNELS; i++)
    {
      const guint16 reading = data[4 + 2 * i] | ((guint16) data[5 + 2 * i] << 8);
      const guint8 thresh = MIN ((reading >> 1) + GOODIX5F10_FDT_DELTA, 0xff);

      self->fdt_base[GOODIX5F10_FDT_THRESH_OFF + 2 * i] = thresh;
      self->fdt_base[GOODIX5F10_FDT_THRESH_OFF + 2 * i + 1] = thresh;
    }
  self->fdt_base_live = TRUE;

  if (bootstrap)
    {
      g_autofree gchar *hex =
        data_to_str (self->fdt_base + GOODIX5F10_FDT_THRESH_OFF,
                     2 * GOODIX5F10_FDT_CHANNELS);

      if (touch_flag != 0)
        fp_warn ("FDT %s: first reading came with touch flag 0x%04x; the thresholds it "
                 "yields may be off until the next capture", tag, touch_flag);
      fp_dbg ("FDT thresholds measured on this sensor (from %s): %s", tag, hex);
    }
}

// FDT-mode and FDT-up only answer once the sensor is clear, so their readings are resting
// levels; FDT-down answers with a finger on it.
static void
check_fdt_mode_cmd (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  if (err) { fpi_ssm_mark_failed (ssm, err); return; }
  dump_fdt_reply ("mode", data, len);
  update_fdt_base (FPI_DEVICE_GOODIXTLS5F10 (dev), "mode", data, len);
  fpi_ssm_next_state (ssm);
}

static void
check_fdt_down_cmd (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  if (err) { fpi_ssm_mark_failed (ssm, err); return; }
  dump_fdt_reply ("down", data, len);
  fpi_ssm_next_state (ssm);
}

static void
check_fdt_up_cmd (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  if (err) { fpi_ssm_mark_failed (ssm, err); return; }
  dump_fdt_reply ("up", data, len);
  update_fdt_base (FPI_DEVICE_GOODIXTLS5F10 (dev), "up", data, len);
  fpi_ssm_next_state (ssm);
}

static void
check_firmware (FpDevice *dev, gchar *firmware, gpointer ssm, GError *error)
{
  if (error) { fpi_ssm_mark_failed (ssm, error); return; }
  fp_dbg ("Device firmware: \"%s\"", firmware);
  if (!g_str_has_prefix (firmware, GOODIX_5F10_FIRMWARE_PREFIX))
    {
      fpi_ssm_mark_failed (ssm, g_error_new (G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                                             "Invalid device firmware: \"%s\"", firmware));
      return;
    }
  if (strcmp (firmware, GOODIX_5F10_FIRMWARE_VERSION) != 0)
    fp_warn ("Device firmware \"%s\" is not the \"%s\" this driver was developed against; "
             "same family, but report anything that misbehaves",
             firmware, GOODIX_5F10_FIRMWARE_VERSION);
  fpi_ssm_next_state (ssm);
}

static void
check_preset_psk (FpDevice *dev, gboolean success, guint32 flags, guint8 *psk,
                  guint16 length, gpointer ssm, GError *error)
{
  g_autofree gchar *psk_str = NULL;

  if (error) { fpi_ssm_mark_failed (ssm, error); return; }
  if (!success)
    {
      fpi_ssm_mark_failed (ssm, g_error_new (G_IO_ERROR, G_IO_ERROR_FAILED,
                                             "Failed to read PSK from device"));
      return;
    }

  // Log only. The chip-side hash is per-device and cannot be computed from the
  // PSK alone without vendor master keys, so we cannot meaningfully compare it
  // here. A wrong PSK is caught later by the TLS-PSK handshake failing.
  psk_str = data_to_str (psk, length);
  fp_dbg ("Device PMK hash: 0x%s (flags 0x%08x)", psk_str, flags);
  if (flags != GOODIX_5F10_PSK_FLAGS || length != GOODIX_5F10_PSK_LEN)
    {
      fpi_ssm_mark_failed (ssm, g_error_new (G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                                             "Unexpected PSK reply: flags=0x%08x len=%u",
                                             flags, length));
      return;
    }
  fpi_ssm_next_state (ssm);
}

static void
check_reset (FpDevice *dev, gboolean success, guint16 number, gpointer ssm, GError *error)
{
  if (error) { fpi_ssm_mark_failed (ssm, error); return; }
  if (!success)
    {
      fpi_ssm_mark_failed (ssm, g_error_new (G_IO_ERROR, G_IO_ERROR_FAILED,
                                             "Failed to reset device"));
      return;
    }
  fp_dbg ("Device reset number: %d", number);
  if (number != GOODIX_5F10_RESET_NUMBER)
    {
      fpi_ssm_mark_failed (ssm, g_error_new (G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                                             "Invalid device reset number: %d", number));
      return;
    }
  fpi_ssm_next_state (ssm);
}

static void
check_success (FpDevice *dev, gboolean success, gpointer ssm, GError *error)
{
  if (error) { fpi_ssm_mark_failed (ssm, error); return; }
  if (!success)
    {
      fpi_ssm_mark_failed (ssm, g_error_new (FP_DEVICE_ERROR, FP_DEVICE_ERROR_PROTO,
                                             "device command reported failure"));
      return;
    }
  fpi_ssm_next_state (ssm);
}

// 5F10 wants ENABLE_CHIP body "01 02"; the generic helper emits "01 00".
static void
send_enable_chip (FpDevice *dev, GoodixNoneCallback callback, gpointer user_data)
{
  guint8 payload[] = {0x01, 0x02};
  GoodixCallbackInfo *cb_info = malloc (sizeof (GoodixCallbackInfo));

  cb_info->callback = G_CALLBACK (callback);
  cb_info->user_data = user_data;
  goodix_send_protocol (dev, GOODIX_CMD_ENABLE_CHIP, payload, sizeof (payload),
                        NULL, TRUE, GOODIX_TIMEOUT, FALSE, goodix_receive_none, cb_info);
}

// Discard wrapper for state results we neither consume nor inspect (protocol nudges
// whose only purpose is to advance the chip's internal state machine -- payload is
// dropped, SSM is advanced).
static void
on_discard_default (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  if (err) { fpi_ssm_mark_failed (ssm, err); return; }
  fp_dbg ("discarding %u-byte reply (state passthrough)", len);
  fpi_ssm_next_state (ssm);
}

// ---- activation SSM (run once at open) ------------------------------------------------

enum activate_states {
  ACTIVATE_READ_AND_NOP,
  ACTIVATE_ENABLE_CHIP,
  ACTIVATE_QUERY_MCU,            // MCU state ping after ENABLE_CHIP (init-sequence nudge)
  ACTIVATE_NOP,
  ACTIVATE_CHECK_FW_VER,
  ACTIVATE_CHECK_PSK,
  ACTIVATE_READ_OTP,
  ACTIVATE_RESET,
  ACTIVATE_SET_MCU_IDLE,
  ACTIVATE_UPLOAD_MCU_CONFIG,
  ACTIVATE_SET_POWERDOWN_SCAN_FREQUENCY,
  ACTIVATE_NUM_STATES,
};

// 32-byte OTP block. Each calibration field is stored with triple redundancy
// (value, its bitwise complement, and a duplicate); validate the triple and fall
// back to chip defaults when it does not hold.
#define GOODIX5F10_OTP_LEN 0x20

static gboolean
otp_triple_ok (guint8 value, guint8 complement, guint8 duplicate)
{
  return value != 0 && value == (guint8) ~complement && value == duplicate;
}

// Read the OTP block during activation (the vendor init sequence does the same) and log
// the per-device factory calibration for diagnostics. Non-fatal: the live baseline-subtract
// path works regardless, so a bad or odd OTP never blocks activation.
static void
parse_otp (FpDevice *dev, guint8 *data, guint16 length, gpointer ssm, GError *error)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  if (error) { fpi_ssm_mark_failed (ssm, error); return; }

  if (length < GOODIX5F10_OTP_LEN)
    {
      fp_warn ("OTP too short (%u < %u); skipping per-device calibration",
               length, (guint) GOODIX5F10_OTP_LEN);
      fpi_ssm_next_state (ssm);
      return;
    }

  self->otp_dac_valid   = otp_triple_ok (data[0x1F], data[0x1B], data[0x1A]);
  self->otp_tcode_valid = otp_triple_ok (data[0x16], data[0x17], data[0x19]);
  self->otp_dac_h = self->otp_dac_valid ? data[0x1F] : 0;
  self->otp_tcode = self->otp_tcode_valid ? data[0x16] : 0;

  if (self->otp_dac_valid)
    fp_dbg ("OTP per-device factory DAC_H = 0x%02x", self->otp_dac_h);
  else
    fp_warn ("OTP DAC_H failed redundancy check; will use chip default");
  if (self->otp_tcode_valid)
    fp_dbg ("OTP per-device factory TCODE = 0x%02x", self->otp_tcode);
  else
    fp_warn ("OTP TCODE failed redundancy check; will use chip default");

  fpi_ssm_next_state (ssm);
}

// ---- MCU config ------------------------------------------------------------------------
// The config template is a dump taken from the development unit, and one of its entries is
// that unit's factory DAC: tag 0x0220 carries (OTP DAC << 4) | 8. Uploading it verbatim
// runs every other sensor at this laptop's operating point, so the entry is rewritten from
// the OTP of the device in hand. Layout: a leading byte, eight (base, size) section pairs,
// then 4-byte entries of u16 tag and u16 value, and the table's own checksum in the tail.
#define GOODIX5F10_CONFIG_SECTIONS 8
#define GOODIX5F10_CONFIG_DAC_TAG  0x0220

static void
config_set_checksum (guint8 *config, gsize len)
{
  guint32 checksum = 0xa5a5;

  for (gsize i = 0; i + 2 < len; i += 2)
    checksum = (checksum + (config[i] | ((guint16) config[i + 1] << 8))) & 0xffff;
  checksum = 0x10000 - checksum;
  config[len - 2] = checksum & 0xff;
  config[len - 1] = (checksum >> 8) & 0xff;
}

static void
config_set_dac (guint8 *config, gsize len, guint8 dac)
{
  const guint16 value = ((guint16) dac << 4) | 8;

  for (guint section = 0; section < GOODIX5F10_CONFIG_SECTIONS; section++)
    {
      const guint base = config[1 + 2 * section];
      const guint end = MIN (base + config[2 + 2 * section], len);

      for (guint entry = base; entry + 4 <= end; entry += 4)
        {
          if ((config[entry] | ((guint16) config[entry + 1] << 8)) != GOODIX5F10_CONFIG_DAC_TAG)
            continue;
          fp_dbg ("config DAC 0x%04x -> 0x%04x (OTP 0x%02x)",
                  (guint) (config[entry + 2] | ((guint16) config[entry + 3] << 8)),
                  (guint) value, dac);
          config[entry + 2] = value & 0xff;
          config[entry + 3] = value >> 8;
        }
    }
  config_set_checksum (config, len);
}

static void
activate_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ACTIVATE_READ_AND_NOP:
      goodix_start_read_loop (dev);
      goodix_send_nop (dev, check_none, ssm);
      break;
    case ACTIVATE_ENABLE_CHIP:
      send_enable_chip (dev, check_none, ssm);
      break;
    case ACTIVATE_QUERY_MCU:
      // Init-sequence nudge. The reply payload (MCU state struct) is not consumed:
      // only firing the command matters so the chip's internal state machine matches
      // what subsequent steps expect.
      goodix_send_query_mcu_state (dev, on_discard_default, ssm);
      break;
    case ACTIVATE_NOP:
      goodix_send_nop (dev, check_none, ssm);
      break;
    case ACTIVATE_CHECK_FW_VER:
      goodix_send_query_firmware_version (dev, check_firmware, ssm);
      break;
    case ACTIVATE_CHECK_PSK:
      goodix_send_preset_psk_read (dev, GOODIX_5F10_PSK_FLAGS, 0, check_preset_psk, ssm);
      break;
    case ACTIVATE_READ_OTP:
      goodix_send_read_otp (dev, parse_otp, ssm);
      break;
    case ACTIVATE_RESET:
      goodix_send_reset (dev, TRUE, 20, check_reset, ssm);
      break;
    case ACTIVATE_SET_MCU_IDLE:
      goodix_send_mcu_switch_to_idle_mode (dev, 20, check_none, ssm);
      break;
    case ACTIVATE_UPLOAD_MCU_CONFIG:
      memcpy (self->config, goodix_5f10_config, sizeof (self->config));
      // GOODIX5F10_NO_DAC_PATCH uploads the template untouched, so a tester can compare
      // both operating points with one build.
      if (self->otp_dac_valid && g_getenv ("GOODIX5F10_NO_DAC_PATCH") == NULL)
        config_set_dac (self->config, sizeof (self->config), self->otp_dac_h);
      goodix_send_upload_config_mcu (dev, self->config, sizeof (self->config), NULL,
                                     check_success, ssm);
      break;
    case ACTIVATE_SET_POWERDOWN_SCAN_FREQUENCY:
      goodix_send_set_powerdown_scan_frequency (dev, 100, check_success, ssm);
      break;
    }
}

static void
open_tls_complete (FpDevice *dev, gpointer user_data, GError *error)
{
  fpi_device_open_complete (dev, error);
}

static void
activate_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  self->task_ssm = NULL;
  if (error)
    {
      fp_err ("failed during activation: %s", error->message);
      fpi_device_open_complete (dev, error);
      return;
    }
  // Bring up the real TLS-PSK channel; open completes when the handshake finishes.
  goodix_tls_init (dev, open_tls_complete, NULL);
}

// ---- capture sub-SSM: one finger press -> one processed 8-bit frame -------------------

enum capture_states {
  CAP_QUERY_MCU,
  CAP_FDT_MODE,
  CAP_CAL_FDT_UP,
  CAP_CAL_NAV0,
  CAP_CAL_GET_IMG,
  CAP_FDT_DOWN,
  CAP_GET_IMG,
  CAP_GET_IMG_REPEAT, // loop CAP_GET_IMG until frames_target frames are collected
  CAP_REARM,          // optional NAV0 between burst frames (only if GOODIX5F10_REARM set)
  CAP_FDT_UP,
  CAP_NUM_STATES,
};

// Re-arm callback: after NAV0, go take the next burst frame (best-effort; ignore NAV0 error).
static void
after_rearm (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  if (err)
    {
      fp_dbg ("capture: NAV0 re-arm error (ignored): %s", err->message);
      g_error_free (err);
    }
  fpi_ssm_jump_to_state (ssm, CAP_GET_IMG);
}

static void
on_baseline_img (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);
  g_autofree Goodix5f10Pix *frame = NULL;
  Goodix5f10Pix min = 0xffff, max = 0;
  guint64 total = 0;
  double mean;

  if (err) { fpi_ssm_mark_failed (ssm, err); return; }
  if (len != GOODIX5F10_RAW_FRAME_SIZE)
    {
      fpi_ssm_mark_failed (ssm, g_error_new (FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                                             "unexpected baseline frame size %u (expected %u)",
                                             len, (guint) GOODIX5F10_RAW_FRAME_SIZE));
      return;
    }

  frame = g_malloc0 (GOODIX5F10_PIXELS * sizeof (Goodix5f10Pix));
  decode_frame (frame, len, data);

  for (guint i = 0; i < GOODIX5F10_PIXELS; i++)
    {
      const Goodix5f10Pix px = frame[i];

      total += px;
      if (px < min) min = px;
      if (px > max) max = px;
    }
  mean = (double) total / GOODIX5F10_PIXELS;
  fp_dbg ("baseline frame: mean %.1f, min %u, max %u", mean, (guint) min, (guint) max);

  if (mean < GOODIX5F10_BASELINE_MIN_MEAN)
    {
      if (self->baseline_retries < GOODIX5F10_BASELINE_RETRIES)
        {
          self->baseline_retries++;
          fp_dbg ("baseline frame carries no signal (mean %.1f); retrying (%u/%u)",
                  mean, self->baseline_retries, (guint) GOODIX5F10_BASELINE_RETRIES);
          fpi_ssm_jump_to_state_delayed (ssm, CAP_CAL_NAV0, GOODIX5F10_BASELINE_RETRY_MS);
          return;
        }
      fpi_ssm_mark_failed (ssm, g_error_new (FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                                             "baseline frame carried no signal after %u attempts",
                                             (guint) GOODIX5F10_BASELINE_RETRIES));
      return;
    }

  g_free (self->calibration_img);
  self->calibration_img = g_steal_pointer (&frame);
  fpi_ssm_next_state (ssm);
}

static void
on_capture_img (FpDevice *dev, guint8 *data, guint16 len, gpointer ssm, GError *err)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  // Tolerate a read error once we already hold at least one frame: the finger was likely
  // lifted mid-burst. Release with FDT_UP and proceed with what we collected.
  if (err)
    {
      if (self->capture_frames && self->capture_frames->len > 0)
        {
          fp_dbg ("capture: tolerated read error after %u frame(s): %s",
                  self->capture_frames->len, err->message);
          g_error_free (err);
          fpi_ssm_jump_to_state (ssm, CAP_FDT_UP);
          return;
        }
      fpi_ssm_mark_failed (ssm, err);
      return;
    }
  if (len != GOODIX5F10_RAW_FRAME_SIZE)
    {
      if (self->capture_frames && self->capture_frames->len > 0)
        {
          fp_dbg ("capture: tolerated short frame %u after %u frame(s)",
                  len, self->capture_frames->len);
          fpi_ssm_jump_to_state (ssm, CAP_FDT_UP);
          return;
        }
      fpi_ssm_mark_failed (ssm, g_error_new (FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID,
                                             "unexpected frame size %u (expected %u)",
                                             len, (guint) GOODIX5F10_RAW_FRAME_SIZE));
      return;
    }

  Goodix5f10Pix *raw = g_malloc0 (GOODIX5F10_PIXELS * sizeof (Goodix5f10Pix));
  decode_frame (raw, len, data);
  if (self->calibration_img)
    linear_subtract_inplace (raw, self->calibration_img, GOODIX5F10_PIXELS);
  guint8 *squashed = g_malloc0 (GOODIX5F10_PIXELS);
  squash_frame_linear (raw, squashed, GOODIX5F10_PIXELS);
  g_free (raw);

  g_ptr_array_add (self->capture_frames, process_to_8bit (squashed));
  g_free (squashed);

  fpi_ssm_next_state (ssm); // -> CAP_GET_IMG_REPEAT
}

static void
capture_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);
  const guint8 *fdt = self->fdt_base;
  const guint16 fdt_len = sizeof (self->fdt_base);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case CAP_QUERY_MCU:
      {
        g_clear_pointer (&self->capture_frames, g_ptr_array_unref);
        self->capture_frames = g_ptr_array_new_with_free_func (g_free);
        self->baseline_retries = 0;
        if (self->frames_target == 0)
          self->frames_target = 1;
        self->rearm_between = (g_getenv ("GOODIX5F10_REARM") != NULL);
      }
      goodix_send_query_mcu_state (dev, check_none_cmd, ssm);
      break;
    case CAP_FDT_MODE:
      goodix_send_mcu_switch_to_fdt_mode (dev, fdt, fdt_len, NULL, check_fdt_mode_cmd, ssm);
      break;
    case CAP_CAL_FDT_UP:
      goodix_send_mcu_switch_to_fdt_up (dev, fdt, fdt_len, NULL, check_fdt_up_cmd, ssm);
      break;
    case CAP_CAL_NAV0:
      goodix_send_nav_0 (dev, check_none_cmd, ssm);
      break;
    case CAP_CAL_GET_IMG:
      goodix_tls_read_image (dev, on_baseline_img, ssm);
      break;
    case CAP_FDT_DOWN:
      goodix_send_mcu_switch_to_fdt_down (dev, fdt, fdt_len, NULL, check_fdt_down_cmd, ssm);
      break;
    case CAP_GET_IMG:
      goodix_tls_read_image (dev, on_capture_img, ssm);
      break;
    case CAP_GET_IMG_REPEAT:
      {
        // Grab more frames from the same press until we hit the target. GET_IMAGE rescans
        // the sensor each call, so normally no FDT re-arm is needed (and re-arming FDT_DOWN on
        // a held finger would block waiting for a down-edge that already happened). If the chip
        // turns out to re-serve one buffer, GOODIX5F10_REARM inserts a NAV0 between grabs.
        if (self->capture_frames->len < self->frames_target)
          fpi_ssm_jump_to_state (ssm, self->rearm_between ? CAP_REARM : CAP_GET_IMG);
        else
          fpi_ssm_jump_to_state (ssm, CAP_FDT_UP);
      }
      break;
    case CAP_REARM:
      goodix_send_nav_0 (dev, after_rearm, ssm);
      break;
    case CAP_FDT_UP:
      goodix_send_mcu_switch_to_fdt_up (dev, fdt, fdt_len, NULL, check_fdt_up_cmd, ssm);
      break;
    }
}

// ---- enroll SSM -----------------------------------------------------------------------

enum enroll_states {
  ENROLL_CAPTURE,
  ENROLL_PROCESS,
  ENROLL_NEXT,
  ENROLL_NUM_STATES,
};

static guint
env_uint (const char *name, guint fallback)
{
  const char *v = g_getenv (name);
  if (!v || !*v)
    return fallback;
  return (guint) g_ascii_strtoull (v, NULL, 10);
}

static void
enroll_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case ENROLL_CAPTURE:
      fpi_ssm_start_subsm (ssm, fpi_ssm_new (dev, capture_run_state, CAP_NUM_STATES));
      break;
    case ENROLL_PROCESS:
      {
        // One sample per touch: pick the highest-quality frame of this press's burst and bank
        // it. We keep EVERY touch that yields a frame above the keypoint gate -- no novelty/
        // overlap drop. Overlap between touches is wanted (it keeps the gallery connected); the
        // center->edges placement that produces it is guided by the UI, not enforced here.
        gint best_idx = -1;
        int  best_kp = -1;
        guint n = self->capture_frames ? self->capture_frames->len : 0;

        for (guint i = 0; i < n; i++)
          {
            guint8 *img = g_ptr_array_index (self->capture_frames, i);
            SigfmImgInfo *info = sigfm_extract_ex (img, GOODIX5F10_WIDTH, GOODIX5F10_HEIGHT, GOODIX5F10_SIGFM_MAX_KP);
            int kp = info ? sigfm_keypoints_count (info) : -1;

            if (kp > best_kp)
              { best_kp = kp; best_idx = (gint) i; }
            if (info)
              sigfm_free_info (info);
          }

        if (best_idx >= 0 && best_kp >= (int) self->enroll_min_kp)
          {
            // Move the chosen frame into the gallery; the rest of the burst is freed when
            // capture_frames is unref'd (its free func is g_free; the stolen slot is NULLed).
            guint8 *keep = g_ptr_array_index (self->capture_frames, best_idx);
            g_ptr_array_index (self->capture_frames, best_idx) = NULL;
            g_ptr_array_add (self->enroll_images, keep);

            self->enroll_touches++;
            self->enroll_ignored = 0;
            guint have = self->enroll_images->len;
            fp_info ("enroll sample %u/%u, touch %u, best %d keypoints",
                     have, self->enroll_target, self->enroll_touches, best_kp);
            fpi_device_enroll_progress (dev, (gint) have, NULL, NULL);
          }
        else
          {
            // Poor contact / not a finger: don't advance the sample count, just re-prompt.
            self->enroll_ignored++;
            fp_dbg ("enroll: touch ignored (%u in a row), best %d keypoints (< %u)",
                    self->enroll_ignored, best_kp, self->enroll_min_kp);
          }

        g_clear_pointer (&self->capture_frames, g_ptr_array_unref);

        if (self->enroll_ignored >= GOODIX5F10_ENROLL_IGNORE_LIMIT)
          {
            fpi_ssm_mark_failed (ssm, g_error_new (FP_DEVICE_ERROR,
                                                   FP_DEVICE_ERROR_DATA_INVALID,
                                                   "too many low-quality scans; check finger "
                                                   "placement and sensor"));
            break;
          }
        fpi_ssm_next_state (ssm);
      }
      break;
    case ENROLL_NEXT:
      // Stop on a fixed count of accepted samples (the industry model); no zone/coverage gate.
      if (self->enroll_images->len >= self->enroll_target)
        {
          fp_info ("enroll complete: %u gallery frames over %u touches",
                   self->enroll_images->len, self->enroll_touches);
          fpi_ssm_mark_completed (ssm);
        }
      else
        fpi_ssm_jump_to_state (ssm, ENROLL_CAPTURE);
      break;
    }
}

static void
enroll_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  self->task_ssm = NULL;
  if (error)
    {
      g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
      g_clear_pointer (&self->capture_frames, g_ptr_array_unref);
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }

  FpPrint *print = NULL;
  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);

  // Store the gallery as GVariant "aay": one fixed 8-bit frame per enrolled zone. We persist
  // raw frames (not serialized SIFT descriptors) on purpose: the template stays independent
  // of the matcher, so a future sigfm/algorithm change still matches old enrollments without
  // re-enrolling. Trade-off (accepted): the print holds raw fingerprint images, and verify
  // re-extracts SIFT each time (cheap at 56x176).
  GVariantBuilder builder;
  g_variant_builder_init (&builder, G_VARIANT_TYPE ("aay"));
  for (guint i = 0; i < self->enroll_images->len; i++)
    {
      guint8 *img = g_ptr_array_index (self->enroll_images, i);
      g_variant_builder_add (&builder, "@ay",
                             g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE, img,
                                                        GOODIX5F10_PIXELS, 1));
    }
  GVariant *data = g_variant_builder_end (&builder);
  g_object_set (G_OBJECT (print), "fpi-data", data, NULL);
  guint gallery_frames = self->enroll_images->len;
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);

  fp_info ("enrollment complete: %u gallery frames over %u touches",
           gallery_frames, self->enroll_touches);
  fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

// ---- verify / identify SSM ------------------------------------------------------------

enum verify_states {
  VERIFY_CAPTURE,
  VERIFY_MATCH,
  VERIFY_NUM_STATES,
};

// Score a set of probe frames (all grabbed from one finger press) against one stored
// template's gallery (a GVariant "aay"). Each gallery sample is extracted once and matched
// against every probe frame; the sample's score is the best over probes. Returns the best
// score across all samples and the count of samples scoring >= THRESHOLD.
static int
score_template_multi (SigfmImgInfo **probes, guint n_probes,
                      GVariant *tmpl_data, int *out_match_count)
{
  GVariantIter iter;
  GVariant *child;
  int best = 0, count = 0;

  g_variant_iter_init (&iter, tmpl_data);
  while ((child = g_variant_iter_next_value (&iter)))
    {
      gsize len;
      const guint8 *img = g_variant_get_fixed_array (child, &len, 1);
      if (len == GOODIX5F10_PIXELS)
        {
          SigfmImgInfo *ti = sigfm_extract_ex (img, GOODIX5F10_WIDTH, GOODIX5F10_HEIGHT, GOODIX5F10_SIGFM_MAX_KP);
          int sample_best = 0;
          for (guint p = 0; p < n_probes; p++)
            {
              if (!probes[p]) continue;
              int score = sigfm_match_score (probes[p], ti);
              if (score > sample_best) sample_best = score;
            }
          sigfm_free_info (ti);
          if (sample_best >= GOODIX5F10_SIGFM_THRESHOLD) count++;
          if (sample_best > best) best = sample_best;
        }
      g_variant_unref (child);
    }
  *out_match_count = count;
  return best;
}

static void
verify_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case VERIFY_CAPTURE:
      fpi_ssm_start_subsm (ssm, fpi_ssm_new (dev, capture_run_state, CAP_NUM_STATES));
      break;

    case VERIFY_MATCH:
      {
        FpiDeviceAction action = fpi_device_get_current_action (dev);

        // Build a probe set from every frame grabbed during this press.
        guint n_probes = self->capture_frames ? self->capture_frames->len : 0;
        SigfmImgInfo **probes = g_new0 (SigfmImgInfo *, MAX (n_probes, 1));
        g_autoptr (GString) kp_line = g_string_new ("SIGFM probe frames keypoints:");
        for (guint i = 0; i < n_probes; i++)
          {
            guint8 *img = g_ptr_array_index (self->capture_frames, i);
            probes[i] = sigfm_extract_ex (img, GOODIX5F10_WIDTH, GOODIX5F10_HEIGHT, GOODIX5F10_SIGFM_MAX_KP);
            g_string_append_printf (kp_line, " %d", sigfm_keypoints_count (probes[i]));
          }
        fp_dbg ("%s", kp_line->str);

        if (action == FPI_DEVICE_ACTION_IDENTIFY)
          {
            GPtrArray *gallery = NULL;
            FpPrint *match = NULL;
            int best_score = 0;

            fpi_device_get_identify_data (dev, &gallery);
            for (guint i = 0; gallery && i < gallery->len; i++)
              {
                FpPrint *tmpl = g_ptr_array_index (gallery, i);
                GVariant *tdata = NULL;
                g_object_get (G_OBJECT (tmpl), "fpi-data", &tdata, NULL);
                if (!tdata) continue;
                int mc = 0;
                int best = score_template_multi (probes, n_probes, tdata, &mc);
                g_variant_unref (tdata);
                if (best >= GOODIX5F10_SIGFM_BEST_MIN &&
                    mc >= GOODIX5F10_SIGFM_MIN_SAMPLES && best > best_score)
                  {
                    best_score = best;
                    match = tmpl;
                  }
              }
            fp_dbg ("identify best sigfm %d (%u probe frames)", best_score, n_probes);
            fpi_device_identify_report (dev, match, NULL, NULL);
          }
        else
          {
            FpPrint *print = NULL;
            GVariant *data = NULL;
            int best = 0, mc = 0;

            fpi_device_get_verify_data (dev, &print);
            g_object_get (G_OBJECT (print), "fpi-data", &data, NULL);
            if (data)
              {
                best = score_template_multi (probes, n_probes, data, &mc);
                g_variant_unref (data);
              }
            fp_dbg ("verify best sigfm %d, matching samples %d over %u probe frames "
                    "(thr %d/best %d/min %d)",
                    best, mc, n_probes, GOODIX5F10_SIGFM_THRESHOLD,
                    GOODIX5F10_SIGFM_BEST_MIN, GOODIX5F10_SIGFM_MIN_SAMPLES);
            if (best >= GOODIX5F10_SIGFM_BEST_MIN && mc >= GOODIX5F10_SIGFM_MIN_SAMPLES)
              fpi_device_verify_report (dev, FPI_MATCH_SUCCESS, NULL, NULL);
            else
              fpi_device_verify_report (dev, FPI_MATCH_FAIL, NULL, NULL);
          }

        for (guint i = 0; i < n_probes; i++)
          if (probes[i]) sigfm_free_info (probes[i]);
        g_free (probes);
        g_clear_pointer (&self->capture_frames, g_ptr_array_unref);
        fpi_ssm_next_state (ssm);
      }
      break;
    }
}

static void
verify_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);
  FpiDeviceAction action = fpi_device_get_current_action (dev);

  self->task_ssm = NULL;
  g_clear_pointer (&self->capture_frames, g_ptr_array_unref);

  if (action == FPI_DEVICE_ACTION_IDENTIFY)
    fpi_device_identify_complete (dev, error);
  else
    fpi_device_verify_complete (dev, error);
}

// ---- FpDevice vfuncs ------------------------------------------------------------------

// Load the 32-byte per-device TLS-PSK from disk into `self`.
// The path is taken from the GOODIX_5F10_PSK_FILE env var when set, otherwise
// from GOODIX_5F10_DEFAULT_PSK_FILE. On failure returns FALSE and `error`
// carries a message that names the missing file and points the user at the
// companion PSK extraction tool.
static gboolean
load_secrets (FpiDeviceGoodixTls5f10 *self, GError **error)
{
  const gchar *psk_path = g_getenv (GOODIX_5F10_PSK_FILE_ENV);
  g_autofree gchar *data = NULL;
  gsize length = 0;
  g_autoptr (GError) inner = NULL;

  if (!psk_path) psk_path = GOODIX_5F10_DEFAULT_PSK_FILE;

  if (!g_file_get_contents (psk_path, &data, &length, &inner) ||
      length != GOODIX_5F10_PSK_LEN)
    {
      g_set_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL,
                   "goodixtls5f10: failed to load TLS-PSK from \"%s\" (%s). "
                   "The 5F10 sensor uses a device-specific TLS-PSK that the chip "
                   "is provisioned with at the factory and cannot be derived from "
                   "scratch on Linux. Place exactly 32 raw bytes at the above "
                   "path (overridable via the %s environment variable). See the "
                   "libfprint goodix-5f10 documentation for the companion "
                   "extraction tool.",
                   psk_path,
                   inner ? inner->message : "wrong file length",
                   GOODIX_5F10_PSK_FILE_ENV);
      return FALSE;
    }
  memcpy (self->tls_psk, data, GOODIX_5F10_PSK_LEN);
  return TRUE;
}

static void
dev_open (FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);
  GError *error = NULL;

  if (!load_secrets (self, &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }

  // goodix_dev_init() returns TRUE on success (it returns g_usb_device_claim_interface).
  if (!goodix_dev_init (dev, &error))
    {
      fpi_device_open_complete (dev, error);
      return;
    }
  // Feed the per-device TLS session PSK to the OpenSSL server callback.
  goodix_tls_set_psk (self->tls_psk, GOODIX_5F10_PSK_LEN);

  self->task_ssm = fpi_ssm_new (dev, activate_run_state, ACTIVATE_NUM_STATES);
  fpi_ssm_start (self->task_ssm, activate_complete);
}

static void
dev_close (FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);
  GError *error = NULL;

  g_clear_pointer (&self->calibration_img, g_free);
  g_clear_pointer (&self->capture_frames, g_ptr_array_unref);
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);

  goodix_reset_state (dev);
  goodix_shutdown_tls (dev, &error);
  goodix_dev_deinit (dev, &error);

  fpi_device_close_complete (dev, error);
}

static void
dev_enroll (FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  self->enroll_touches = 0;
  self->enroll_ignored = 0;
  self->frames_target = GOODIX5F10_ENROLL_FRAMES;
  g_clear_pointer (&self->enroll_images, g_ptr_array_unref);
  self->enroll_images = g_ptr_array_new_with_free_func (g_free);
  g_clear_pointer (&self->calibration_img, g_free);

  // Fixed-count enroll knobs (env-overridable for live FAR/FRR tuning without a rebuild).
  self->enroll_target = env_uint ("GOODIX5F10_ENROLL_SAMPLES", GOODIX5F10_ENROLL_SAMPLES);
  self->enroll_min_kp = env_uint ("GOODIX5F10_ENROLL_MIN_KP", GOODIX5F10_ENROLL_MIN_KP);
  fp_dbg ("enroll: collecting %u samples (min %u keypoints/frame)",
          self->enroll_target, self->enroll_min_kp);

  self->task_ssm = fpi_ssm_new (dev, enroll_run_state, ENROLL_NUM_STATES);
  fpi_ssm_start (self->task_ssm, enroll_complete);
}

static void
dev_verify (FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  self->frames_target = GOODIX5F10_VERIFY_FRAMES;
  g_clear_pointer (&self->calibration_img, g_free);

  self->task_ssm = fpi_ssm_new (dev, verify_run_state, VERIFY_NUM_STATES);
  fpi_ssm_start (self->task_ssm, verify_complete);
}

static void
dev_identify (FpDevice *dev)
{
  FpiDeviceGoodixTls5f10 *self = FPI_DEVICE_GOODIXTLS5F10 (dev);

  self->frames_target = GOODIX5F10_VERIFY_FRAMES;
  g_clear_pointer (&self->calibration_img, g_free);

  self->task_ssm = fpi_ssm_new (dev, verify_run_state, VERIFY_NUM_STATES);
  fpi_ssm_start (self->task_ssm, verify_complete);
}

static void
dev_cancel (FpDevice *dev)
{
  // Abort the in-flight USB transfer via the base transport's cancel token (the driver has
  // no transfers of its own). Without this, a cancel while waiting for a finger is a no-op.
  goodix_cancel (dev);
}

static void
fpi_device_goodixtls5f10_init (FpiDeviceGoodixTls5f10 *self)
{
  // Seed with the development unit's table so the first FDT command has something to
  // send; update_fdt_base replaces it with this sensor's own on the first reply.
  memcpy (self->fdt_base, fdt_reference_thresholds, sizeof (self->fdt_base));
}

static void
fpi_device_goodixtls5f10_class_init (FpiDeviceGoodixTls5f10Class *class)
{
  FpiDeviceGoodixTlsClass *gx_class = FPI_DEVICE_GOODIXTLS_CLASS (class);
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (class);

  gx_class->interface = GOODIX_5F10_INTERFACE;
  gx_class->ep_in = GOODIX_5F10_EP_IN;
  gx_class->ep_out = GOODIX_5F10_EP_OUT;

  dev_class->id = "goodixtls5f10";
  dev_class->full_name = "Goodix TLS Fingerprint Sensor 5F10";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = id_table;
  // Fixed number of enrollment samples; progress (gallery size) maps 1:1 onto the UI bar.
  dev_class->nr_enroll_stages = GOODIX5F10_ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1; // small sensor, no thermal throttle
  dev_class->features = FP_DEVICE_FEATURE_VERIFY | FP_DEVICE_FEATURE_IDENTIFY;

  dev_class->open = dev_open;
  dev_class->close = dev_close;
  dev_class->enroll = dev_enroll;
  dev_class->verify = dev_verify;
  dev_class->identify = dev_identify;
  dev_class->cancel = dev_cancel;
}
