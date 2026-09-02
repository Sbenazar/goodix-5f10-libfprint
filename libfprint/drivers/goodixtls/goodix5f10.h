/*
 * Goodix TLS driver for libfprint -- chip 27c6:5F10 (Milan/ST411SEC family).
 *
 * Copyright (C) 2026 libfprint contributors
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

// Per-chip constants verified empirically on the live 5F10:
//   - psk_flags 0xbb020003 (same family as the 511)
//   - reset_number 1024 (511 reports 2048)
//   - firmware "GF_ST411SEC_APP_12705"
//   - config table: the vendor's default 5F10 profile, 224 bytes
//   - geometry: 56 px scan width x 176 scanlines (9856 px; RAW_FRAME_SIZE == 14797)
//
// The TLS-PSK session key is NOT compiled in. It is device-specific and loaded at
// activation time from disk (see the PSK_FILE path below and load_secrets()).

#pragma once

#define GOODIX_5F10_INTERFACE (0)
#define GOODIX_5F10_EP_IN (0x1 | FPI_USB_ENDPOINT_IN)
#define GOODIX_5F10_EP_OUT (0x1 | FPI_USB_ENDPOINT_OUT)

// The firmware this driver was developed against, and the family it belongs to. Only
// the family is enforced: 12704 is in the field and speaks the same protocol, and
// refusing it just sends people editing this line (github issue #2).
#define GOODIX_5F10_FIRMWARE_VERSION ("GF_ST411SEC_APP_12705")
#define GOODIX_5F10_FIRMWARE_PREFIX  ("GF_ST411SEC_APP_")

#define GOODIX_5F10_PSK_FLAGS (0xbb020003)

// The live 5F10 reports reset number 1024 (the 511 reports 2048). Empirical, from
// the chip's reset(0xa2) reply during bring-up.
#define GOODIX_5F10_RESET_NUMBER (1024)

// The session PSK is 32 bytes.
#define GOODIX_5F10_PSK_LEN (32)

// Default disk location. Contains raw 32 bytes (no encoding, no header).
// Overridable via the GOODIX_5F10_PSK_FILE env var (mainly for tests and per-user installs).
#define GOODIX_5F10_DEFAULT_PSK_FILE "/var/lib/fprint/goodix-5f10/psk"
#define GOODIX_5F10_PSK_FILE_ENV     "GOODIX_5F10_PSK_FILE"

// The factory values of the unit this driver was written on. Only GOODIX5F10_NO_OTP_PATCH
// uses them, to put the sensor back on the operating point every build before this one ran
// at -- handy when a tester wants to compare the two without swapping binaries.
#define GOODIX5F10_DEV_DAC   (0x2a)
#define GOODIX5F10_DEV_TCODE (0xfa)

// The MCU config table, its layout and the three per-unit entries: see goodix5f10.c.
#define GOODIX5F10_CONFIG_LEN 224

// The table the driver uploads, before any per-device rewriting. len is set to its size.
const guint8 *goodix5f10_config_template (gsize *len);

// Rewrite the three per-unit entries of a copy of that table from one device's OTP bytes
// and fix the table checksum.
void goodix5f10_config_patch_from_otp (guint8 *config, gsize len, guint8 dac, guint8 tcode);
