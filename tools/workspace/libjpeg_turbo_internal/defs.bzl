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
        fail("The BUILD file at `drake/third_party/com_github_tensorflow_tensorflow/third_party/jpeg/jpeg.BUILD` that Drake uses to compile jpeg-turbo has begun passing `includes` to its cc_library rule(s). You'll need to investigate what changed during an upgrade and decide how to adjust this macro to deal with the change.")  # noqa
    kwargs = amend(kwargs, "includes", default = ["."])
    kwargs = amend(kwargs, "copts", append = ["-fvisibility=hidden"])
    kwargs["linkstatic"] = True
    _cc_library(**kwargs)
