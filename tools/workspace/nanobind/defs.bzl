load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

# This matches our minimum supported Python version of >= 3.12.
LIMITED_API_DEFINES = ["Py_LIMITED_API=0x030C0000"]

# cc_library_with_limited_api compiles is dependencies with Py_LIMITED_API set
# in their copts.
_builder = with_cfg(cc_library)
_builder.extend("copt", [
    "-D" + definition
    for definition in LIMITED_API_DEFINES
])
cc_library_with_limited_api, _ = _builder.build()
