# Transition definitions for libpng. Thes are loaded in
# //tools/skylark:cc_hidden.

LIBPNG_COPTS = [
   "-w",
    # Turn off <config.h> guessing. It should be implicitly off by default,
    # but it would be a disaster if the default somehow didn't work.
    "-DPNG_NO_CONFIG_H=1",
] + select({
   "@platforms//cpu:x86_64": ["-msse4.1"],
})

LIBPNG_LABEL_FLAGS = {
    "@module_libpng//:pnglibconf_options": ["-PNG_READ_eXIf_SUPPORTED"],
}

LIBPNG_LOCAL_DEFINES = select({
    "@platforms//cpu:x86_64": ["PNG_INTEL_SSE_IMPLEMENTATION=3"],
    "//conditions:default": [],
})
