load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

# See https://docs.python.org/3/c-api/stable.html#c.Py_LIMITED_API.
# This matches our minimum supported Python version of >= 3.12.
LIMITED_API_DEFINES = ["Py_LIMITED_API=0x030C0000"]

# cc_library_with_limited_api compiles its dependencies with Py_LIMITED_API set
# in their copts.
_builder = with_cfg(cc_library)
_builder.extend("copt", [
    "-D" + definition
    for definition in LIMITED_API_DEFINES
])
cc_library_with_limited_api, _ = _builder.build()
