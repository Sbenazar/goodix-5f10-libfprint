// Goodix Tls driver for libfprint

// Copyright (C) 2021 Alexander Meiler <alex.meiler@protonmail.com>
// Copyright (C) 2021 Matthieu CHARETTE <matthieu.charette@gmail.com>
// Copyright (C) 2021 Natasha England-Elbro <ashenglandelbro@protonmail.com>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA

#pragma once

#define GOODIX_EP_IN_MAX_BUF_SIZE (0x10000)
#define GOODIX_EP_OUT_MAX_BUF_SIZE (0x40)

#define GOODIX_NULL_CHECKSUM (0x88)

#define GOODIX_FLAGS_MSG_PROTOCOL (0xa0)
#define GOODIX_FLAGS_TLS (0xb0)

// Opcode encoding. Each command is parameterised by two nibbles (cmd0, cmd1) and a
// "wait for ack" flag; the byte that actually goes on the wire is:
//
//     cmd_byte = (cmd0 << 4) | (cmd1 << 1);
//     if (timeout == 0)                 // "fire-and-forget" / no-reply send
//         cmd_byte |= GOODIX_CMD_NOREPLY_BIT;
//
// This explains the low-bit pairs in the vendor driver's command set
// (0x00<->0x01, 0x96<->0x97, 0xae<->0xaf, 0xd0<->0xd1, 0xd4<->0xd5): the same
// command, with bit 0 distinguishing "wait for chip reply" (clear) from
// "no-reply / fire-and-forget" (set). The chip-side semantics are identical;
// the bit only controls whether the host waits for an ack.
#define GOODIX_CMD_NOREPLY_BIT (0x01)

// ---- Command opcodes ----
// Each constant is the BASE (reply-expected) form; the "_NO_REPLY" variants below
// are the same command with GOODIX_CMD_NOREPLY_BIT set.

#define GOODIX_CMD_NOP (0x00)                          // (cmd0=0, cmd1=0) keep-alive / warmup padding
#define GOODIX_CMD_MCU_GET_IMAGE (0x20)                // (cmd0=2, cmd1=0) request a fingerprint frame
#define GOODIX_CMD_MCU_SWITCH_TO_FDT_DOWN (0x32)       // (cmd0=3, cmd1=1) FDT-down (wait for finger)
#define GOODIX_CMD_MCU_SWITCH_TO_FDT_UP (0x34)         // (cmd0=3, cmd1=2) FDT-up (wait for release)
#define GOODIX_CMD_MCU_SWITCH_TO_FDT_MODE (0x36)       // (cmd0=3, cmd1=3) FDT base/mode setup
#define GOODIX_CMD_NAV_0 (0x50)                        // (cmd0=5, cmd1=0) navigation mode / re-arm
#define GOODIX_CMD_MCU_SWITCH_TO_IDLE_MODE (0x70)      // (cmd0=7, cmd1=0) idle / sleep
#define GOODIX_CMD_WRITE_SENSOR_REGISTER (0x80)        // (cmd0=8, cmd1=0) write sensor register
#define GOODIX_CMD_READ_SENSOR_REGISTER (0x82)         // (cmd0=8, cmd1=1) read sensor register
#define GOODIX_CMD_UPLOAD_CONFIG_MCU (0x90)            // (cmd0=9, cmd1=0) upload MCU config blob
#define GOODIX_CMD_SET_POWERDOWN_SCAN_FREQUENCY (0x94) // (cmd0=9, cmd1=2)
#define GOODIX_CMD_ENABLE_CHIP (0x96)                  // (cmd0=9, cmd1=3) DriverState handshake
#define GOODIX_CMD_RESET (0xa2)                        // (cmd0=10,cmd1=1) reset MCU
#define GOODIX_CMD_READ_OTP (0xa6)                     // (cmd0=10,cmd1=3) read OTP block
#define GOODIX_CMD_FIRMWARE_VERSION (0xa8)             // (cmd0=10,cmd1=4) get FW / sector0 version
#define GOODIX_CMD_QUERY_MCU_STATE (0xae)              // (cmd0=10,cmd1=7) MCU state ping
#define GOODIX_CMD_ACK (0xb0)                          // ack frame marker
#define GOODIX_CMD_REQUEST_TLS_CONNECTION (0xd0)       // (cmd0=13,cmd1=0) start TLS-PSK handshake
#define GOODIX_CMD_TLS_SUCCESSFULLY_ESTABLISHED (0xd4) // (cmd0=13,cmd1=2) confirm TLS up
#define GOODIX_CMD_PRESET_PSK_READ (0xe4)              // (cmd0=14,cmd1=2) read provisioning PSK

// ---- No-reply / fire-and-forget variants (low bit set) ----
// These send the SAME command on the chip side, but the host does NOT block for the
// reply. Used for warmup padding and for TLS bring-up nudges whose response arrives
// via a separate path (e.g. the TLS handshake itself, or the next state poll).
#define GOODIX_CMD_NOP_KEEPALIVE \
  (GOODIX_CMD_NOP | GOODIX_CMD_NOREPLY_BIT)                    // 0x01 -- warmup-burst NOP
#define GOODIX_CMD_QUERY_MCU_STATE_NO_REPLY \
  (GOODIX_CMD_QUERY_MCU_STATE | GOODIX_CMD_NOREPLY_BIT)        // 0xaf -- state ping w/o ack
#define GOODIX_CMD_REQUEST_TLS_NO_REPLY \
  (GOODIX_CMD_REQUEST_TLS_CONNECTION | GOODIX_CMD_NOREPLY_BIT) // 0xd1 -- trigger TLS, no ack
#define GOODIX_CMD_TLS_ESTABLISHED_NO_REPLY \
  (GOODIX_CMD_TLS_SUCCESSFULLY_ESTABLISHED | GOODIX_CMD_NOREPLY_BIT) // 0xd5 -- switch to encrypted stream
#define GOODIX_CMD_ENABLE_CHIP_NO_REPLY \
  (GOODIX_CMD_ENABLE_CHIP | GOODIX_CMD_NOREPLY_BIT)            // 0x97 -- DriverState w/o ack

typedef struct __attribute__((__packed__)) _GoodixPack
{
  guint8 flags;
  guint16 length;
} GoodixPack;

typedef struct __attribute__((__packed__)) _GoodixProtocol
{
  guint8 cmd;
  guint16 length;
} GoodixProtocol;

typedef struct __attribute__((__packed__)) _GoodixAck
{
  guint8 cmd;
  guint8 always_true : 1;
  guint8 has_no_config : 1;
  guint8 : 6;
} GoodixAck;

typedef struct __attribute__((__packed__)) _GoodixNop
{
  guint32 unknown;
} GoodixNop;

typedef struct __attribute__((__packed__)) _GoodixMcuSwitchToIdleMode
{
  guint8 sleep_time;
  guint8 : 8;
} GoodixMcuSwitchToIdleMode;

typedef struct __attribute__((__packed__)) _GoodixWriteSensorRegister
{
  guint8 multiples;
  guint16 address;
  guint16 value;
} GoodixWriteSensorRegister;

typedef struct __attribute__((__packed__)) _GoodixReadSensorRegister
{
  guint8 multiples;
  guint16 address;
  guint8 length;
  guint8 : 8;
} GoodixReadSensorRegister;

typedef struct __attribute__((__packed__)) _GoodixSetPowerdownScanFrequency
{
  guint16 powerdown_scan_frequency;
} GoodixSetPowerdownScanFrequency;

typedef struct __attribute__((__packed__)) _GoodixEnableChip
{
  guint8 enable;
  guint8 : 8;
} GoodixEnableChip;

typedef struct __attribute__((__packed__)) _GoodixReset
{
  guint8 reset_sensor : 1;
  guint8 soft_reset_mcu : 1;
  guint8 : 6;
  guint8 sleep_time;
} GoodixReset;

typedef struct __attribute__((__packed__)) _GoodixQueryMcuState
{
  guint8 unused_flags;
} GoodixQueryMcuState;

typedef struct __attribute__((__packed__)) _GoodixPresetPsk
{
  guint32 flags;
  guint32 length;
} GoodixPresetPsk;

typedef struct __attribute__((__packed__)) _GoodixDefault
{
  guint8 unused_flags;
  guint8 : 8;
} GoodixDefault;

typedef struct __attribute__((__packed__)) _GoodixNone
{
  guint16 : 16;
} GoodixNone;

guint8 goodix_calc_checksum (guint8 *data,
                             guint16 length);

void goodix_encode_pack (guint8   flags,
                         guint8  *payload,
                         guint16  payload_len,
                         gboolean pad_data,
                         guint8 **data,
                         guint32 *data_len);

void goodix_encode_protocol (guint8        cmd,
                             const guint8 *payload,
                             guint16       payload_len,
                             gboolean      calc_checksum,
                             gboolean      pad_data,
                             guint8      **data,
                             guint32      *data_len);

gboolean goodix_decode_pack (guint8   *data,
                             guint32   data_len,
                             guint8   *flags,
                             guint8  **payload,
                             guint16  *payload_len,
                             gboolean *valid_checksum);

gboolean goodix_decode_protocol (guint8   *data,
                                 guint32   data_len,
                                 guint8   *cmd,
                                 guint8  **payload,
                                 guint16  *payload_len,
                                 gboolean *valid_checksum,
                                 gboolean *valid_null_checksum);
