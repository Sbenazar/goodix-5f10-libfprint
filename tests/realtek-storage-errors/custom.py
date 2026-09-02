#!/usr/bin/env python3

# Deleting a print that is not in the sensor's storage makes the realtek driver
# fail its task SSM, which is the path that used to hand an already-freed GError
# to fpi_device_delete_complete(). Reading the reported error is therefore the
# point of this test: before the fix it read freed memory, and the GError was
# freed a second time when the GTask finalized.
#
# The print below is a stored realtek print, serialized, whose template is
# deliberately not on the device.
#
# The same session then enrolls a finger and enrolls it again, checking that the
# duplicate is reported as FP_DEVICE_ERROR_DATA_DUPLICATE rather than PROTO.

import base64
import traceback
import sys
import gi

gi.require_version('FPrint', '2.0')
from gi.repository import FPrint, GLib

# Exit with error on any exception, included those happening in async callbacks
sys.excepthook = lambda *args: (traceback.print_exception(*args), sys.exit(1))

# A stored print for user 'testuser', right index finger, user id
# 'FP1-20260101-7-DEADBEEF-test'.
STORED_PRINT = base64.b64decode(
    'RlAzAQAAAHJlYWx0ZWsAMAABB3Rlc3R1c2VyAABGUDEtMjAyNjAxMDEtNy1ERUFEQkVFRi10ZXN0AAAhSQsAAAAAAP9GUDEtMjAyNjAxMDEtNy1ERUFEQkVFRi10ZXN0ACh5YXkpAHZAOBoODA==')

ctx = GLib.main_context_default()

c = FPrint.Context()
c.enumerate()
devices = c.get_devices()

d = devices[0]
del devices

assert d.get_driver() == "realtek"
assert d.has_feature(FPrint.DeviceFeature.STORAGE)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_DELETE)
assert d.has_feature(FPrint.DeviceFeature.STORAGE_CLEAR)

d.open_sync()

# Make sure the device holds no templates, so the print below cannot match.
d.clear_storage_sync()

p = FPrint.Print.deserialize(STORED_PRINT)
assert p.get_driver() == "realtek"
assert p.get_device_stored()
assert p.get_finger() == FPrint.Finger.RIGHT_INDEX

print("deleting a print that is not in the device storage")
try:
    d.delete_print_sync(p)
except GLib.Error as error:
    print("delete reported: {}".format(error.message))
    assert error.matches(FPrint.DeviceError.quark(), FPrint.DeviceError.PROTO)
    assert error.message
else:
    assert False, "deleting a template that is not stored should fail"
del p
print("delete attempt done")

# Storage is still empty, so enrolling the same finger twice must report the
# error class expected by fprintd.

def enroll_progress(*args):
    print('enroll progress: ' + str(args))

template = FPrint.Print.new(d)

print("enrolling")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
p = d.enroll_sync(template, None, enroll_progress, None)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
print("enroll done")

print("enrolling duplicate")
template = FPrint.Print.new(d)
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
try:
    d.enroll_sync(template, None, enroll_progress, None)
except GLib.Error as error:
    assert error.matches(FPrint.DeviceError.quark(),
                         FPrint.DeviceError.DATA_DUPLICATE)
else:
    raise AssertionError("Duplicate enrollment unexpectedly succeeded")
assert d.get_finger_status() == FPrint.FingerStatusFlags.NONE
print("duplicate enrollment rejected")

d.delete_print_sync(p)
d.close_sync()

del d
del c
