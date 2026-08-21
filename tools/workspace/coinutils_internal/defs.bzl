load("//tools/skylark:cc.bzl", "cc_library")
load("//tools/skylark:drake_cc.bzl", "cc_linkonly_library")
load("//tools/workspace:cmake_configure_file.bzl", "autoconf_configure_file")
load(
    "//tools/workspace:vendor_cxx.bzl",
    "cc_library_vendored",
    "generate_vendor_patch",
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
    """Defines a static cc_library for code from a COIN-OR project.
    (See https://github.com/coin-or and related.)

    Args:
        name: The output cc_library name.
        config_h: Filename assumed by the C++ when including the config header;
            e.g., `FooConfig.h`.
        config_h_public: Source file to supply `FooConfig.h` for downstream
            projects; e.g., `Foo/src/config_ipopt_default.h`.
        config_h_private: Autoconf template file to supply `FooConfig.h` when
            building our srcs; e.g., `Foo/src/config.h.in`.
        autoconf_defines: Definitions we want to use for `config.h.in`;
            refer to autoconf_configure_file() for detailed semantics.
        autoconf_undefines: Definitions we do NOT want from `config.h.in`;
            refer to autoconf_configure_file() for detailed semantics.
        hdrs_public: Source files to provide for downstream projects.
        includes_public: the cc_library `includes = ...` for the hdrs_public.
        hdrs_private: Source files to provide when building our srcs.
        includes_private: the cc_library `includes = ...` for the hdrs_private.
        srcs: C++ sources to compile.
        visibility: The visibility of the resulting cc_library. (All other
            rules will use default visibility.)
        deps: The deps required for all hdrs and/or srcs.
        config_private_defines: Additional preprocessor definitions to blend
            into the config_h header.
        vendor_tool_args: Extra flags for the source-file vendoring tool;
            refer to cc_library_vendored() for detailed semantics.
        output_vendoring_patch: (Optional) Filename to create a `*.patch` file
            that contains all of our rule's source file edits.
    """

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
    cc_library(
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
    cc_library(
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
        isystem = True,
        vendor_tool_args = vendor_tool_args,
        linkstatic = True,
        copts = [
            "-w",
            # On Clang 12, "-w" doesn't suppress this for some reason.
            "-Wno-register",
            # The Coin family of software uses NDEBUG for gross things, not
            # just controlling <cassert> but actually inserting huge swaths
            # of debugging code, some of which is not thread-safe. Even in
            # Drake's debug builds, we still don't want any of that stuff.
            "-DNDEBUG",
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
        isystem = True,
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
    if output_vendoring_patch:
        before = [config_h_private]
        after = ["hdr_private/" + config_h]
        for file in srcs + hdrs_private + hdrs_public:
            if file not in before:
                before.append(file)
                after.append(file.replace("src/", "drake_src/"))
        generate_vendor_patch(
            name = output_vendoring_patch,
            srcs = before,
            srcs_vendored = after,
            extra_prologue = """
This patch also shows the config.h settings used by Drake.""",
        )
