/*
 * Goodix 5F10 config table unit tests
 * Copyright (C) 2026 libfprint contributors
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <glib.h>

#include "drivers/goodixtls/goodix5f10.h"

/* The table the sensor in this laptop runs, captured off the vendor exchange. Patching the
 * shipped template with this unit's OTP bytes has to land on it byte for byte, checksum
 * included -- that is the whole claim about how the three per-unit entries are derived. */
static const guint8 dev_config[] = {
  0x30, 0x11, 0x64, 0x75, 0x00, 0x75, 0x2c, 0xa1, 0x1c, 0xbd, 0x18, 0xd5,
  0x00, 0xd5, 0x00, 0xd5, 0x00, 0xba, 0x00, 0x00, 0x80, 0xca, 0x00, 0x06,
  0x00, 0x84, 0x00, 0xbe, 0xb2, 0x86, 0x00, 0xc5, 0xb9, 0x88, 0x00, 0xb5,
  0xad, 0x8a, 0x00, 0x9d, 0x95, 0x8c, 0x00, 0x00, 0xbe, 0x8e, 0x00, 0x00,
  0xc5, 0x90, 0x00, 0x00, 0xb5, 0x92, 0x00, 0x00, 0x9d, 0x94, 0x00, 0x00,
  0xaf, 0x96, 0x00, 0x00, 0xbf, 0x98, 0x00, 0x00, 0xb6, 0x9a, 0x00, 0x00,
  0xa7, 0xd2, 0x00, 0x00, 0x00, 0xd4, 0x00, 0x00, 0x00, 0xd6, 0x00, 0x00,
  0x00, 0xd8, 0x00, 0x00, 0x00, 0x12, 0x00, 0x03, 0x04, 0xd0, 0x00, 0x00,
  0x00, 0x70, 0x00, 0x00, 0x00, 0x72, 0x00, 0x78, 0x56, 0x74, 0x00, 0x34,
  0x12, 0x20, 0x00, 0x10, 0x40, 0x20, 0x02, 0xa8, 0x02, 0x2a, 0x01, 0x82,
  0x03, 0x22, 0x00, 0x01, 0x20, 0x24, 0x00, 0x14, 0x00, 0x80, 0x00, 0x01,
  0x04, 0x5c, 0x00, 0x00, 0x01, 0x56, 0x00, 0x0c, 0x24, 0x58, 0x00, 0x05,
  0x00, 0x32, 0x00, 0x08, 0x02, 0x66, 0x00, 0x00, 0x02, 0x7c, 0x00, 0x00,
  0x38, 0x82, 0x00, 0x80, 0x19, 0x2a, 0x01, 0x08, 0x00, 0x5c, 0x00, 0x80,
  0x00, 0x54, 0x00, 0x00, 0x01, 0x62, 0x00, 0x38, 0x04, 0x64, 0x00, 0x10,
  0x00, 0x66, 0x00, 0x00, 0x02, 0x7c, 0x00, 0x01, 0x38, 0x2a, 0x01, 0x08,
  0x00, 0x5c, 0x00, 0x00, 0x01, 0x52, 0x00, 0x08, 0x00, 0x54, 0x00, 0x00,
  0x01, 0x66, 0x00, 0x00, 0x02, 0x7c, 0x00, 0x01, 0x38, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xcb, 0xfd};

/* Deliberately not the driver's own reader: a copy of the code under test would agree with
 * it however wrong both are. */
static guint16
entry_value (const guint8 *config, guint section, guint16 tag)
{
  const guint base = config[1 + 2 * section];
  const guint end = base + config[2 + 2 * section];

  for (guint entry = base; entry + 4 <= end; entry += 4)
    if ((config[entry] | (guint16) config[entry + 1] << 8) == tag)
      return config[entry + 2] | (guint16) config[entry + 3] << 8;

  g_error ("no tag 0x%04x in section %u", tag, section);
}

static guint16
table_checksum (const guint8 *config, gsize len)
{
  guint32 sum = 0xa5a5;

  for (gsize i = 0; i + 2 < len; i += 2)
    sum = (sum + (config[i] | (guint16) config[i + 1] << 8)) & 0xffff;

  return 0x10000 - sum;
}

static void
test_config_this_unit (void)
{
  g_autofree guint8 *config = NULL;
  const guint8 *template;
  gsize len;

  template = goodix5f10_config_template (&len);
  g_assert_cmpuint (len, ==, sizeof (dev_config));
  config = g_memdup2 (template, len);

  goodix5f10_config_patch_from_otp (config, len, GOODIX5F10_DEV_DAC, GOODIX5F10_DEV_TCODE);
  g_assert_cmpmem (config, len, dev_config, sizeof (dev_config));
}

/* The FMI-76 from github issue #1 reports DAC 0x3c and TCODE 0xf9. Nothing here has been
 * seen on that sensor -- it is the arithmetic pinned down so a later capture can disagree
 * with it. */
static void
test_config_other_unit (void)
{
  g_autofree guint8 *config = NULL;
  const guint8 *template;
  gsize len;

  template = goodix5f10_config_template (&len);
  config = g_memdup2 (template, len);

  goodix5f10_config_patch_from_otp (config, len, 0x3c, 0xf9);
  g_assert_cmphex (entry_value (config, 0, 0x0220), ==, 0x03c8);
  g_assert_cmphex (entry_value (config, 4, 0x005c), ==, 0x0100);
  g_assert_cmphex (entry_value (config, 2, 0x0082), ==, 0x1680);

  /* Tag 0x005c also sits in sections 2 and 3 and neither is calibration, so both have to
   * come out of the patch untouched. */
  g_assert_cmphex (entry_value (config, 2, 0x005c), ==, 0x0100);
  g_assert_cmphex (entry_value (config, 3, 0x005c), ==, 0x0080);

  g_assert_cmphex (table_checksum (config, len), ==,
                   (config[len - 2] | (guint16) config[len - 1] << 8));
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/goodix5f10/config/this-unit", test_config_this_unit);
  g_test_add_func ("/goodix5f10/config/other-unit", test_config_other_unit);

  return g_test_run ();
}
