# libfprint - Goodix 27c6:5F10 driver

A libfprint driver for the Goodix `27c6:5F10` fingerprint sensor - the patch-style
reader that ships in HONOR MagicBook laptops and shows up as
`USB\VID_27C6&PID_5F10`. This repo is a full libfprint tree with one extra driver
on top, sitting on current
[libfprint](https://gitlab.freedesktop.org/libfprint/libfprint) master (1.94.100);
everything outside `libfprint/drivers/goodixtls/` is stock.

> **Status: builds, not yet tested on real hardware.** I wrote and reverse-engineered
> this against my own machine's traffic, but I don't have a second 5F10 to confirm
> enroll/verify accross devices. If you own one of these sensors, a testing report is
> exactly what I'm after before this goes upstream - see [Testing](#testing).

## The sensor

The 5F10 is a tiny 56x176 image sensor. It is too small for NBIS minutiae matching
(the same wall the Goodix HTK32 hits), so the driver is not a normal
`FpImageDevice`. Instead it:

- talks to the chip over the goodixtls TLS-PSK transport (OpenSSL),
- captures raw frames and matches them host-side with sigfm (FAST-9 + BRIEF-256 + RANSAC),
- keeps a small gallery of raw frames in the `FpPrint`.

That "FpDevice + host-side sigfm + raw-frame gallery" shape is taken from AndyHazz's
goodix53x5 driver - see [Credits](#credits).

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

You need meson, ninja, glib, gusb, udev and OpenSSL headers (on Ubuntu that is
`meson ninja-build libglib2.0-dev libgusb-dev libssl-dev systemd-dev`). Built here with
meson 1.11 and gcc 16; CI runs the same two commands on Ubuntu on every push.

```sh
git clone https://github.com/Sbenazar/goodix-5f10-libfprint
cd goodix-5f10-libfprint
meson setup build -Ddrivers=goodixtls5f10 -Ddoc=false -Dgtk-examples=false -Dintrospection=false
ninja -C build
```

`./build/libfprint/fprint-list-supported-devices` should then list `27c6:5f10`.

## Install

Building is not enough. fprintd loads libfprint from the system library path, so
the build has to end up there, and `meson setup` needs to be told where that is -
the default prefix puts it in `/usr/local/lib64`, which is not in `ld.so.conf` on
Fedora, and nothing ever picks it up (thanks @nexplorer-3e for that one):

```sh
meson setup build -Dprefix=/usr -Dlibdir=/usr/lib64 -Ddoc=false -Dgtk-examples=false
ninja -C build && sudo ninja -C build install
```

There's deliberately no `-Ddrivers=goodixtls5f10` in that line. It's fine while
you're testing out of the build tree, but a libfprint built with it contains this
driver and nothing else, so installing that leaves your system unable to talk to
any other fingerprint sensor.

One more trap, since I walked into it myself: do not park a backup copy next to
the installed library. `libfprint-2.so.2.0.0.bak` carries the same SONAME as the
real one, and `ldconfig` is free to point `libfprint-2.so.2` at the backup - after
which you're running the old build and wondering why your changes do nothing.
Keep backups outside the library path.

## Enroll & verify

```sh
# 1. put the recovered PSK in place (see goodix-5f10-psk)
sudo systemctl restart fprintd
fprintd-enroll
fprintd-verify
```

Under SELinux fprintd also has to be allowed to read the PSK, or activation fails
with a permission error:

```sh
sudo chown root:root /var/lib/fprint/goodix-5f10/psk
sudo chcon -t fprintd_var_lib_t /var/lib/fprint/goodix-5f10/psk
sudo systemctl restart fprintd
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
