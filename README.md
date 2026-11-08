# libfprint - Goodix 27c6:5F10 driver

A libfprint driver for the Goodix `27c6:5F10` fingerprint sensor - the patch-style
reader that ships in HONOR MagicBook laptops and shows up as
`USB\VID_27C6&PID_5F10`. This repo is a full libfprint tree with one extra driver
on top, sitting on current
[libfprint](https://gitlab.freedesktop.org/libfprint/libfprint) master (1.94.100);
everything outside `libfprint/drivers/goodixtls/` is stock.

> **Status: no longer only my machine.** Three other people have run it, on a
> MagicBook 2023 (GLO-G52), an X14 Plus (FMI-76) and a DRA-XX. More reports are still
> what I'm after before this goes upstream - see [Testing](#testing).

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

## Per-device calibration

Every 5F10 leaves the factory with its own operating point and keeps it in OTP: an
analog offset for the read path, and a code that sets the integration time. Three
entries of the config table the driver uploads carry it.

Builds before this one shipped that table as it came off my laptop, so every other
sensor ran at my operating point - on a DRA-XX, a baseline of 2993 counts where its own
OTP asks for 2396. The driver reads the OTP during activation now and rewrites those
three.

Prints from an older build should survive. I checked first, since the gallery is raw
frames: enrolled at my own DAC, then verified with the config forced to two other
people's values. 11 of 14 through at one, 5 of 6 at the other, 11 of 14 for the control
- one finger and that few touches, but nothing I could pick out. New enrollments also
carry a stamp of the point they were taken at, so a later surprise turns up in the log
instead of as a finger that quietly stops working.

Re-enrolling is worth it anyway, just not for the reason you might think. The old print
keeps working; what a fresh one gets you is a gallery recorded at the point your sensor
is actually calibrated for, plus the stamp, so if recognition ever does go odd later
there's something in the log to look at. How much better it is I honestly don't know -
my own measurement couldn't tell the two apart.

`GOODIX5F10_NO_OTP_PATCH=1` puts the sensor back on mine if you want to compare.

## The catch: per-device PSK

The 5F10 only speaks TLS, and only with a 32-byte PSK that the sensor and the host
share. That key is not in this code. There are two ways to get one:

- Windows is still on the disk (dual-boot, or at least the partition left in place):
  recover the key it already holds. Nothing gets written to the sensor, so if you
  can go this way, go this way.
- Linux only: the driver can generate a key and write it to the sensor itself. It's
  off unless you ask for it.

Without a key file the driver fails activation with a message that says what to do.

### Windows + Linux: recover the key

-> **[goodix-5f10-psk](https://github.com/Sbenazar/goodix-5f10-psk)**

It reads the key offline and read-only off the Windows partition and drops it at
`/var/lib/fprint/goodix-5f10/psk`.

### Linux only: write your own key

Turn provisioning on for fprintd, enroll once, turn it off again:

```sh
sudo mkdir -p /etc/systemd/system/fprintd.service.d
printf '[Service]\nEnvironment=GOODIX5F10_AUTOPROVISION=1\n' | \
  sudo tee /etc/systemd/system/fprintd.service.d/provision.conf
sudo systemctl daemon-reload
sudo systemctl restart fprintd

fprintd-enroll

sudo rm /etc/systemd/system/fprintd.service.d/provision.conf
sudo systemctl daemon-reload
sudo systemctl restart fprintd
```

On that first open the driver generates a key, writes it with command `0xE0`, reads
it back and saves it to `/var/lib/fprint/goodix-5f10/psk` once the read-back matches.

Worth knowing before you do it:

This replaces whatever key the sensor had. If you might still want the Windows one,
recover it first.

If a write bricks the sensor there's no way back: there's no 5F10 firmware image
anywhere to recover it. So far this has been run on two machines, both on firmware
`GF_ST411SEC_APP_12705`, and the driver refuses to write on anything else.

Don't leave the variable set: with it on, any later mismatch rewrites the key without
asking.

On dual-boot the Windows driver uses the same key slot. On the one machine it was
tried on ([issue #5](https://github.com/Sbenazar/goodix-5f10-libfprint/issues/5))
booting Windows changed the key, and each side rewrote it on mismatch. Without the
variable, on 12705 Linux just stops with an error naming the mismatch.

## Build

You need meson, ninja, glib, gusb, udev and OpenSSL headers (on Ubuntu that is
`meson ninja-build libglib2.0-dev libgusb-dev libssl-dev systemd-dev`). Built here with
meson 1.11 and gcc 16; CI runs the same two commands on Ubuntu on every push.

That list is enough for the driver-only build below. A full build, which is what
you want for Install, also pulls in the other drivers and needs
`libpixman-1-dev libcairo2-dev libgudev-1.0-dev` on top - without them `meson
setup` stops at "pixman is required for aes3500" or "udev is required for SPI
support" (thanks @dimaCaptain).

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
# Fedora and other lib64 distros
meson setup build -Dprefix=/usr -Dlibdir=/usr/lib64 -Ddoc=false -Dgtk-examples=false
# Debian and Ubuntu
meson setup build -Dprefix=/usr -Dlibdir=/usr/lib/x86_64-linux-gnu -Ddoc=false -Dgtk-examples=false

ninja -C build && sudo ninja -C build install
```

If you're unsure which one you're on, `ldconfig -p | grep libfprint-2.so.2` prints
the path the loader actually uses, and that's the directory your build has to land
in.

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
# 1. have a key in place (see "The catch" above)
sudo systemctl restart fprintd
fprintd-enroll
fprintd-verify
```

If you put the key there by hand, under SELinux fprintd also has to be allowed to
read it, or activation fails with a permission error:

```sh
sudo chown root:root /var/lib/fprint/goodix-5f10/psk
sudo chcon -t fprintd_var_lib_t /var/lib/fprint/goodix-5f10/psk
sudo systemctl restart fprintd
```

One thing to know before you reach for `examples/verify` instead: it asks which
finger to verify *after* opening the device, and it reads that answer straight
from stdin inside a main loop callback. While the loop sits there, glib doesn't
refresh the time it hands out to timeouts, so the next command the driver sends
gets a deadline that already expired and fails on the spot with
`Command timed out: 0xae`. It looks like the sensor went stale during the pause;
it didn't, and the same pause with the loop left running is fine. fprintd never
blocks its loop, so nothing you do through fprintd runs into this.

## Testing

This is where you come in. I'm gathering reports before opening an upstream MR.
If you have a 5F10:

1. `lsusb | grep 27c6:5f10` to confirm the sensor.
2. Build (above), then get a key one of the two ways described in "The catch".
3. Enroll, verify.
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
- Linux-side key provisioning from [mounkastel](https://github.com/mounkastel);
- the `FpDevice` host-side-matching **architecture** from
  [AndyHazz/goodix53x5-libfprint](https://github.com/AndyHazz/goodix53x5-libfprint).

## License

LGPL-2.1-or-later, same as libfprint.
