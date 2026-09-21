load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

_builder = with_cfg(cc_library)
_builder.extend(
    "copt",
   [
       "-fvisibility=hidden",
       "-w",
        # Turn off <config.h> guessing. It should be implicitly off by default,
        # but it would be a disaster if the default somehow didn't work.
        "-DPNG_NO_CONFIG_H=1",
   ] + select({
       "@platforms//cpu:x86_64": ["-msse4.1"],
   })
)
_builder.extend("features", ["-supports_dynamic_linker"])
_builder.set(
    Label("@module_libpng//:pnglibconf_options"),
    ["-PNG_READ_eXIf_SUPPORTED"],
)
_png_cc_library_impl, _ = _builder.build()

def png_cc_library(name, **kwargs):
    _png_cc_library_impl(
        name = name,
        local_defines = select({
            "@platforms//cpu:x86_64": ["PNG_INTEL_SSE_IMPLEMENTATION=3"],
            "//conditions:default": [],
        }),
        **kwargs,
    )
