load("//tools/skylark:cc.bzl", _cc_library = "cc_library")
load("//tools/skylark:kwargs.bzl", "amend")

def cc_library(**kwargs):
    """The BUILD file for @libjpeg_turbo_internal uses this macro as its
    implementation of cc_library. This version serves several purposes:

    (1) Forces Drake's canonical @rules_cc import.

    (2) Adjusts the include paths to avoid accidentally using the host OS copy
    of libjpeg from /usr/include/.

    (3) Adjusts the build options to Drake's defaults (hidden, static).
    """
    if kwargs.get("includes"):
        fail("Something in tensorflow changed that needs investigation.")
    kwargs = amend(kwargs, "includes", default = ["."])
    kwargs = amend(kwargs, "copts", append = ["-fvisibility=hidden"])
    kwargs["linkstatic"] = True
    _cc_library(**kwargs)
