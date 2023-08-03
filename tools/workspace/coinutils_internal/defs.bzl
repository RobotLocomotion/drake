load(
    "@drake//tools/skylark:drake_cc.bzl",
    "cc_linkonly_library",
)
load(
    "@drake//tools/workspace:cmake_configure_file.bzl",
    "autoconf_configure_file",
)
load(
    "@drake//tools/workspace:vendor_cxx.bzl",
    "cc_library_vendored",
)

def coin_cc_library(
        *,
        name,
        config_h,
        config_h_public,
        config_h_private,
        autoconf_defines,
        autoconf_undefines,
        hdrs_public,
        includes_public,
        hdrs_private,
        includes_private,
        srcs,
        visibility,
        deps = [],
        config_private_defines = [],
        vendor_tool_args = [],
        output_vendoring_patch = None):
    # Configure the private flavor of FooConfig.h.
    #
    # The upstream FooConfig.h is a tricksy little beast. When compiling the
    # library source code, it refers to the configure-generated header. When
    # installing into include paths for the user, it uses something different
    # (a narrower header with just the version numbers & etc).
    #
    # Here we'll generate the *private* header with the configuration we want.
    #
    # We can use cc_library (not cc_library_vendored) to declare the header
    # because it only has preprocessor definitions (no C++ object code).
    autoconf_configure_file(
        name = "_configure",
        src = config_h_private,
        out = "hdr_private/" + config_h,
        defines = autoconf_defines,
        undefines = autoconf_undefines,
        strict = True,
    )
    native.cc_library(
        name = "_config_private",
        hdrs = [":hdr_private/" + config_h],
        strip_include_prefix = "hdr_private",
        defines = config_private_defines,
        linkstatic = True,
    )

    # The next two rules are for the public flavor of FooConfig.h.
    #
    # We can use cc_library (not cc_library_vendored) to declare the header
    # because it only has preprocessor definitions (no C++ object code).
    native.genrule(
        name = "_genrule_config_public",
        srcs = [config_h_public],
        outs = ["hdr_public/" + config_h],
        cmd = "cp $< $@",
    )
    native.cc_library(
        name = "_config_public",
        hdrs = [":hdr_public/" + config_h],
        strip_include_prefix = "hdr_public",
        linkstatic = True,
    )

    # Compile all of the object code for the library.
    cc_library_vendored(
        name = "_build",
        srcs = srcs,
        srcs_vendored = [
            x.replace("src/", "drake_src/")
            for x in srcs
        ],
        hdrs = hdrs_private,
        hdrs_vendored = [
            x.replace("src/", "drake_src/")
            for x in hdrs_private
        ],
        includes = [
            x.replace("src/", "drake_src/")
            for x in includes_private
        ],
        vendor_tool_args = vendor_tool_args,
        linkstatic = True,
        copts = [
            "-w",
            # On Clang 12, "-w" doesn't suppress this for some reason.
            "-Wno-register",
        ],
        deps = deps + [":_config_private"],
    )

    # Discard the headers, leaving only the object code.
    cc_linkonly_library(
        name = "_objs",
        deps = [":_build"],
    )

    # Assemble the public headers + object code into the library for Drake.
    cc_library_vendored(
        name = name,
        hdrs = hdrs_public,
        hdrs_vendored = [
            x.replace("src/", "drake_hdr/")
            for x in hdrs_public
        ],
        includes = [
            x.replace("src/", "drake_hdr/")
            for x in includes_public
        ],
        vendor_tool_args = vendor_tool_args,
        linkstatic = True,
        deps = deps + [
            ":_config_public",
            ":_objs",
        ],
        visibility = visibility,
    )

    # Optionally, create a patch with all of our source code edits. Sometimes,
    # LICENSE conditions require us to redistribute our source code changes.
    if output_vendoring_patch == None:
        return

    diff_inputs = [
        (config_h_private, "hdr_private/" + config_h),
    ] + zip(
        srcs + hdrs_private,
        [
            x.replace("src/", "drake_src/")
            for x in srcs + hdrs_private
        ],
    )

    for upstream_src, vendor_src in diff_inputs:
        native.genrule(
            name = "_genrule_{}_patch".format(vendor_src),
            srcs = [upstream_src, vendor_src],
            outs = [vendor_src + ".patch"],
            cmd = " ".join([
                "(diff -u0",
                "--label={upstream_src} $(execpath {upstream_src})",
                "--label={vendor_src} $(execpath {vendor_src})",
                "> $@ || [[ $$? == 1 ]])",
            ]).format(
                upstream_src = upstream_src,
                vendor_src = vendor_src,
            ),
        )

    native.genrule(
        name = "_genrule_full_patch",
        srcs = [
            vendor_src + ".patch"
            for (_, vendor_src) in diff_inputs
        ],
        outs = [output_vendoring_patch],
        cmd = "cat $(SRCS) > $@",
    )
