# libfprint -- Goodix 27c6:5F10 driver

A libfprint driver for the Goodix `27c6:5F10` fingerprint sensor -- the patch-style
reader that ships in HONOR MagicBook laptops and shows up as
`USB\VID_27C6&PID_5F10`. This repo is a full libfprint tree with one extra driver
on top; everything else is stock
[libfprint](https://gitlab.freedesktop.org/libfprint/libfprint).

> **Status: builds, not yet tested on real hardware.** I wrote and reverse-engineered
> this against my own machine's traffic, but I don't have a second 5F10 to confirm
> enroll/verify accross devices. If you own one of these sensors, a testing report is
> exactly what I'm after before this goes upstream -- see [Testing](#testing).

## The sensor

The 5F10 is a tiny 56x176 image sensor. It is too small for NBIS minutiae matching
(the same wall the Goodix HTK32 hits), so the driver is not a normal
`FpImageDevice`. Instead it:

- talks to the chip over the goodixtls TLS-PSK transport (OpenSSL),
- captures raw frames and matches them host-side with sigfm (FAST-9 + BRIEF-256 + RANSAC),
- keeps a small gallery of raw frames in the `FpPrint`.

That "FpDevice + host-side sigfm + raw-frame gallery" shape is taken from AndyHazz's
goodix53x5 driver -- see [Credits](#credits).

| | |
|---|---|
| Sensor | Goodix `27c6:5F10` (ST411SEC), 56x176 |
| Firmware | `GF_ST411SEC_APP_12705` |
| Laptops | HONOR MagicBook family |
| Matcher | pure-C sigfm (FAST-9 + BRIEF-256 + RANSAC), no OpenCV |
| Transport | goodixtls, TLS 1.2 PSK (OpenSSL) |

## The catch: per-device PSK

The 5F10 only speaks TLS, and only with a per-device PSK that the factory
provisions. That key is not in this code and cannot be derived from scratch on
Linux. On a dual-boot machine you can recover the one Windows already holds with
the companion tool:

-> **[goodix-5f10-psk](https://github.com/Sbenazar/goodix-5f10-psk)**

It reads the key offline and read-only off the Windows partition and drops it at
`/var/lib/fprint/goodix-5f10/psk`. Without that file the driver fails activation
with a clear message telling you where to put it.

## Build

Verified with meson 1.11 / gcc 16 / libfprint 1.94.5:

```sh
meson setup build -Ddrivers=goodixtls5f10 -Ddoc=false -Dgtk-examples=false -Dintrospection=false
ninja -C build
```

`./build/libfprint/fprint-list-supported-devices` should then list `27c6:5f10`.

## Enroll & verify

```sh
# 1. put the recovered PSK in place (see goodix-5f10-psk)
sudo systemctl restart fprintd
fprintd-enroll
fprintd-verify
```

## Testing

This is where you come in. I'm gathering reports before opening an upstream MR.
If you have a 5F10:

1. `lsusb | grep 27c6:5f10` to confirm the sensor.
2. Recover the PSK with [goodix-5f10-psk](https://github.com/Sbenazar/goodix-5f10-psk).
3. Build (above), enroll, verify.
4. Paste your `G_MESSAGES_DEBUG=all fprintd` log into libfprint issue
   [#735](https://gitlab.freedesktop.org/libfprint/libfprint/-/issues/735), or open
   an issue here. Good or bad, both are useful.

## Credits

This stands on a stack of prior work:

- the **goodixtls** TLS transport and protocol base from
  [goodix-fp-linux-dev](https://github.com/goodix-fp-linux-dev/libfprint);
- the pure-C **sigfm** port from
  [buxel](https://github.com/buxel/libfprint-27c6-5110), which in turn comes from
  the original sigfm authors (Matthieu Charette, Natasha England-Elbro, Timur
  Mangliev);
- the `FpDevice` host-side-matching **architecture** from
  [AndyHazz/goodix53x5-libfprint](https://github.com/AndyHazz/goodix53x5-libfprint).

## License

LGPL-2.1-or-later, same as libfprint.
