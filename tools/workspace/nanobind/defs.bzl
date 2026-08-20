load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

NANOBIND_DEFINES = [
    # See https://docs.python.org/3/c-api/stable.html#c.Py_LIMITED_API.
    # This matches our minimum supported Python version of >= 3.12.
    "Py_LIMITED_API=0x030C0000",

    # See https://nanobind.readthedocs.io/en/latest/refleaks.html#reference-leaks
    # This limits the scope of setting nanobind global flags.
    "NB_DOMAIN=pydrake",
]

# cc_library_with_limited_api compiles its dependencies with Py_LIMITED_API set
# in their copts.
_builder = with_cfg(cc_library)
_builder.extend("copt", [
    "-D" + definition
    for definition in NANOBIND_DEFINES
])
cc_library_with_limited_api, _ = _builder.build()
