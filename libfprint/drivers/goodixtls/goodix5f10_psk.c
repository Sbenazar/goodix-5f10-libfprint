/*
 * Goodix 5F10 TLS-PSK packing for libfprint.
 *
 * Fixed constant of the sensor protocol; identical on every host.
 * The self-test anchors at the bottom of this file and the vector checks in
 * tests/test-goodix5f10-config.c are derived from this implementation; the
 * real-device confirmation is that the chip accepted a packed key written
 * this way and answered the 0xE4 read with a hash that matched SHA256 of the
 * same TLV-1 body (see goodix5f10.c).
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
 */

#include "goodix5f10_psk.h"

#include <string.h>

#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <openssl/hmac.h>
#include <openssl/rand.h>
#include <openssl/sha.h>

/*
 * Static material. Fixed constant of the sensor protocol; identical on every host.
 */

/*
 * kdf48: the 48-byte key block the vendor driver derives for the PSK-WB
 * wrapper (Windows 5F10 driver, gfusb.dll 1.1.127.6; wrapper RVA 0x6BD0,
 * KDF RVA 0x5B70).  The KDF is HMAC-SHA256 in counter mode over a fixed
 * 32-byte seed block built from "Goodix" + 0A 0E 0D 06 16 04:
 *
 *   out = HMAC-SHA256(key_block, BE32(1) || "kgoodwixg\0" ||
 *                     "kaelrgnoerlithm" || 00 00 01 80)
 *       || HMAC-SHA256(key_block, BE32(2) || ...)
 *   kdf48 = out[0:48]
 *
 * Bytes [0:32] are the AES-256-GCM key, [16:48] the outer HMAC-SHA256 key
 * (16-byte overlap).  The same derivation/constant is used by the GDIX51C0
 * vendor driver (gfspi.dll, FUN_180005b90), so this is a family-wide
 * protocol constant, not a per-device secret.  It is not HKDF.
 */
static const guint8 kdf48[48] = {
  0x58, 0xf0, 0x13, 0xee, 0xb7, 0xe2, 0x16, 0xe2,
  0xc1, 0xcd, 0x0e, 0x8f, 0xff, 0xa7, 0xdb, 0xd6,
  0x79, 0x9c, 0xd9, 0x2a, 0xa1, 0x49, 0x01, 0x3e,
  0xa0, 0xe9, 0xf9, 0xfd, 0x7d, 0xc6, 0xd9, 0x4e,
  0x3e, 0xf6, 0x3a, 0x75, 0x98, 0xc2, 0xa4, 0x93,
  0x31, 0x29, 0xe8, 0x71, 0xea, 0x03, 0x04, 0x3b,
};

static const guint8 aad[16] = {
  0x52, 0x2d, 0xc1, 0xf0, 0x99, 0x56, 0x7d, 0x07,
  0xf4, 0x7f, 0x37, 0xa3, 0x2a, 0x84, 0x42, 0x7d,
};

#define MAGIC        (0xff02)
#define PSK_LEN      (32)
#define TLV1_TAG     (0xbb010003u)
#define TLV2_TAG     (0xbb010002u)

static gboolean
sha256 (const guint8 *data, gsize len, guint8 out[SHA256_DIGEST_LENGTH])
{
  unsigned int outlen = 0;
  return EVP_Digest (data, len, out, &outlen, EVP_sha256 (), NULL) == 1
         && outlen == SHA256_DIGEST_LENGTH;
}

gboolean
goodix5f10_psk_tlv1_body (const guint8 psk[32], guint8 out[GOODIX5F10_TLV1_BODY_LEN])
{
  guint8 header[6];
  guint8 ivbuf[6 + 8 + 64]; /* header(6) || PSK[0:8](8) || 16 x LE32(3)(64) */
  guint8 ivhash[SHA256_DIGEST_LENGTH];
  guint8 iv[16];
  guint8 ct[PSK_LEN + 16]; /* +16 so EncryptFinal has room even though GCM emits none */
  guint8 tag[16];
  guint8 outer[SHA256_DIGEST_LENGTH];
  EVP_CIPHER_CTX *ctx;
  int len = 0, flen = 0;
  unsigned int maclen = 0;
  guint8 mac_key[32];
  guint8 hm_in[6 + PSK_LEN + 16];
  gboolean ok;

  /* header = LE16(MAGIC) || LE32(32) */
  header[0] = MAGIC & 0xff;
  header[1] = (MAGIC >> 8) & 0xff;
  header[2] = PSK_LEN & 0xff;
  header[3] = (PSK_LEN >> 8) & 0xff;
  header[4] = (PSK_LEN >> 16) & 0xff;
  header[5] = (PSK_LEN >> 24) & 0xff;

  /*
   * IV = SHA256(header || PSK[0:8] || LE32(3) x 16)[0:16]
   * The IV uses 16 little-endian dwords of 3; that is 64 bytes.
   */
  memcpy (ivbuf, header, sizeof (header));
  memcpy (ivbuf + sizeof (header), psk, 8);
  memset (ivbuf + sizeof (header) + 8, 0, 64); /* LE32(3) x 16, explicit LE bytes */
  for (int i = 0; i < 16; i++)
    ivbuf[sizeof (header) + 8 + i * 4] = 3;
  ok = sha256 (ivbuf, sizeof (ivbuf), ivhash);

  if (ok)
    {
      memcpy (iv, ivhash, sizeof (iv));

      /* AES-256-GCM(key = kdf48[0:32], iv, aad, psk) -> ct(32) || tag(16) */
      memset (ct, 0, sizeof (ct));
      memset (tag, 0, sizeof (tag));
      ctx = EVP_CIPHER_CTX_new ();
      ok = ctx != NULL
           && EVP_EncryptInit_ex (ctx, EVP_aes_256_gcm (), NULL, NULL, NULL) == 1
           && EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_SET_IVLEN, sizeof (iv), NULL) == 1
           && EVP_EncryptInit_ex (ctx, NULL, NULL, kdf48, iv) == 1
           && EVP_EncryptUpdate (ctx, NULL, &len, aad, sizeof (aad)) == 1
           && EVP_EncryptUpdate (ctx, ct, &len, psk, PSK_LEN) == 1
           && EVP_EncryptFinal_ex (ctx, ct + len, &flen) == 1
           && EVP_CIPHER_CTX_ctrl (ctx, EVP_CTRL_GCM_GET_TAG, sizeof (tag), tag) == 1;
      if (ctx != NULL)
        EVP_CIPHER_CTX_free (ctx);
    }

  /* outer = HMAC-SHA256(kdf48[16:48], header || ct || tag) */
  if (ok)
    {
      memcpy (mac_key, kdf48 + 16, sizeof (mac_key));
      memcpy (hm_in, header, sizeof (header));
      memcpy (hm_in + sizeof (header), ct, PSK_LEN);
      memcpy (hm_in + sizeof (header) + PSK_LEN, tag, sizeof (tag));
      ok = HMAC (EVP_sha256 (), mac_key, sizeof (mac_key), hm_in, sizeof (hm_in),
                 outer, &maclen) != NULL
           && maclen == SHA256_DIGEST_LENGTH;
    }

  /* body = outer(32) || header(6) || iv(16) || ct(32) || tag(16) = 102.
   * Copied only once every step above succeeded, so a failure never emits
   * a zero or partial body and `out` is not even written on FALSE. */
  if (ok)
    {
      memcpy (out, outer, 32);
      memcpy (out + 32, header, sizeof (header));
      memcpy (out + 38, iv, 16);
      memcpy (out + 54, ct, PSK_LEN);
      memcpy (out + 86, tag, sizeof (tag));
    }

  OPENSSL_cleanse (ivbuf, sizeof (ivbuf));
  OPENSSL_cleanse (mac_key, sizeof (mac_key));
  OPENSSL_cleanse (hm_in, sizeof (hm_in));

  return ok;
}

gboolean
goodix5f10_psk_host_hash (const guint8 psk[32], guint8 out[GOODIX5F10_PSK_HASH_LEN])
{
  guint8 body[GOODIX5F10_TLV1_BODY_LEN];
  gboolean ok;

  /* A failed pack never reaches the digest: `body` is not read at all then,
   * and FALSE propagates so the caller aborts. */
  ok = goodix5f10_psk_tlv1_body (psk, body) && sha256 (body, sizeof (body), out);
  /* body carries the wrapped key material; do not leave it on the stack */
  OPENSSL_cleanse (body, sizeof (body));
  return ok;
}

gboolean
goodix5f10_autoprovision_enabled (const char *value)
{
  if (value == NULL)
    return FALSE;
  if (strcmp (value, "1") == 0)
    return TRUE;
  if (strcmp (value, "0") != 0)
    /* Plain g_warning, not fp_warn: this file stays free of fpi headers so
     * it can be linked from the unit test. The variable name is spelled out
     * to match GOODIX_5F10_AUTOPROVISION_ENV in goodix5f10.c. */
    g_warning ("ignoring unexpected GOODIX5F10_AUTOPROVISION=%s; only \"1\" "
               "enables automatic PSK provisioning", value);
  return FALSE;
}

Goodix5f10PskDecision
goodix5f10_psk_decide (gboolean success, guint32 flags, guint16 length,
                       const guint8 *chip_hash, gsize chip_len,
                       const guint8 stored[GOODIX5F10_PSK_HASH_LEN],
                       gboolean autoprovision)
{
  guint8 want[GOODIX5F10_PSK_HASH_LEN];

  /* The chip answers any unreadable slot (empty, wrong tag) with status != 0,
   * which the transport reports as success == FALSE. */
  if (!success)
    return GOODIX5F10_PSK_MISSING;

  if (flags != GOODIX5F10_PSK_SLOT_FLAGS || length != GOODIX5F10_PSK_SLOT_LEN
      || chip_hash == NULL || chip_len < GOODIX5F10_PSK_HASH_LEN)
    return GOODIX5F10_PSK_REPLY_INVALID;

  if (!goodix5f10_psk_host_hash (stored, want))
    return GOODIX5F10_PSK_IO_ERROR;

  if (memcmp (want, chip_hash, GOODIX5F10_PSK_HASH_LEN) == 0)
    return GOODIX5F10_PSK_MATCH;

  return autoprovision ? GOODIX5F10_PSK_MISMATCH_AUTOPROVISION
                       : GOODIX5F10_PSK_MISMATCH_FAIL;
}

guint8 *
goodix5f10_psk_random_psk (void)
{
  guint8 *psk = g_malloc (PSK_LEN);

  if (RAND_bytes (psk, PSK_LEN) != 1)
    {
      g_free (psk);
      return NULL;
    }
  return psk;
}

guint8 *
goodix5f10_psk_build_write (const guint8 psk[32], gsize sealed_len,
                            gsize *out_len)
{
  guint8 body[GOODIX5F10_TLV1_BODY_LEN];
  guint8 seed[8];
  guint32 tmp;
  gsize total, off;
  guint8 *out;

  if (!goodix5f10_psk_tlv1_body (psk, body))
    return NULL;

  if (RAND_bytes (seed, sizeof (seed)) != 1)
    {
      OPENSSL_cleanse (body, sizeof (body));
      return NULL;
    }

  /*
   * [0xBB010002][sealed_len+8][sealed][seed(8)][0xBB010003][102][body]
   * then zero-padded to a multiple of 4.
   */
  total = 4 + 4 + sealed_len + 8 + 4 + 4 + sizeof (body);
  total = (total + 3) & ~(gsize) 3;

  out = g_malloc0 (total);
  off = 0;

  tmp = GUINT32_TO_LE (TLV2_TAG);
  memcpy (out + off, &tmp, 4);
  off += 4;

  tmp = GUINT32_TO_LE ((guint32) (sealed_len + 8));
  memcpy (out + off, &tmp, 4);
  off += 4;

  /* Host-backup section.  The MCU stores it verbatim and never reads it, so
   * random bytes are fine. */
  if (sealed_len > 0 && RAND_bytes (out + off, (int) sealed_len) != 1)
    {
      g_free (out);
      OPENSSL_cleanse (body, sizeof (body));
      return NULL;
    }
  off += sealed_len;

  memcpy (out + off, seed, sizeof (seed));
  off += sizeof (seed);

  tmp = GUINT32_TO_LE (TLV1_TAG);
  memcpy (out + off, &tmp, 4);
  off += 4;

  tmp = GUINT32_TO_LE (sizeof (body));
  memcpy (out + off, &tmp, 4);
  off += 4;

  memcpy (out + off, body, sizeof (body));
  off += sizeof (body);

  g_assert (off <= total);

  if (out_len != NULL)
    *out_len = total;

  OPENSSL_cleanse (body, sizeof (body));
  OPENSSL_cleanse (seed, sizeof (seed));
  return out;
}

/* ------------------------------------------------------------------ */
/* Known-answer self test. The anchors below are derived from this    */
/* implementation; the real-device check is that the chip accepted    */
/* the write and the read-back hash matched (see goodix5f10.c).       */
/* ------------------------------------------------------------------ */

static const guint8 anchor_zeros[GOODIX5F10_PSK_HASH_LEN] = {
  0xb5, 0xe0, 0xbe, 0xeb, 0x94, 0xc8, 0x4e, 0xb9,
  0x9b, 0x88, 0x3a, 0xbd, 0x5c, 0x25, 0x10, 0x73,
  0xc5, 0x6b, 0x91, 0x03, 0x5c, 0x56, 0x2a, 0x91,
  0xa4, 0x6c, 0x7f, 0x33, 0x49, 0xc3, 0x6c, 0x89,
};

static const guint8 anchor_seq[GOODIX5F10_PSK_HASH_LEN] = {
  0xd1, 0xba, 0x9a, 0x47, 0x90, 0xf8, 0xd4, 0x7b,
  0xa8, 0xb5, 0x00, 0x43, 0xd7, 0x2b, 0x57, 0x7c,
  0xdc, 0x2d, 0x1b, 0x70, 0x1d, 0x46, 0x61, 0x97,
  0xaa, 0x74, 0x1a, 0x94, 0xfd, 0x0f, 0x1e, 0xc4,
};

gboolean
goodix5f10_psk_selftest (void)
{
  guint8 psk[PSK_LEN];
  guint8 got[GOODIX5F10_PSK_HASH_LEN];
  gboolean ok = TRUE;

  memset (psk, 0, sizeof (psk));
  ok &= goodix5f10_psk_host_hash (psk, got)
        && memcmp (got, anchor_zeros, sizeof (got)) == 0;

  for (int i = 0; i < PSK_LEN; i++)
    psk[i] = (guint8) i;
  ok &= goodix5f10_psk_host_hash (psk, got)
        && memcmp (got, anchor_seq, sizeof (got)) == 0;

  return ok;
}
