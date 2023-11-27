load("//tools/skylark:cc.bzl", "cc_library")

def cc_wrap_static_archive_hidden(
        name,
        *,
        static_archive_name,
        visibility = ["//visibility:private"]):
    """Wraps a static library (an `*.a` file) to use hidden linker visibility,
    i.e., the library does not export any public symbols. On Linux, we can do
    that with a linker flag. On macOS, we need to copy and modify the archive.

    The `static_archive_name` refers to the library being wrapped, which
    cannot contain any bazel package qualifiers (i.e.., `//foo:bar` is not
    valid; only bare names like `bar` are valid).
    """
    for char in ["/", ":", "."]:
        if char in static_archive_name:
            fail("The static_archive_name must be a plain name, not a label")

    # On macOS, we need to use a helper tool to copy the archive.
    osx_tool = "@drake//tools/skylark:rewrite_osx_ar_hidden"
    osx_archive_name = "_{}_osx.a".format(name)
    native.genrule(
        name = "_{}_genrule".format(name),
        tools = [osx_tool],
        srcs = [static_archive_name],
        outs = [osx_archive_name],
        cmd = " ".join([
            "$(location {})".format(osx_tool),
            "--input",
            "$(location :{})".format(static_archive_name),
            "--output",
            "$(location :{})".format(osx_archive_name),
        ]),
        tags = [
            # Only run this when necessary, not as part of ":all".
            "manual",
        ],
        visibility = ["//visibility:private"],
    )

    # Provide a cc_library with the platform-specific settings.
    cc_library(
        name = name,
        srcs = select({
            "@drake//tools/cc_toolchain:linux": [],
            "@drake//tools/cc_toolchain:apple": [
                osx_archive_name,
            ],
        }),
        deps = select({
            "@drake//tools/cc_toolchain:linux": [
                ":{}".format(static_archive_name),
            ],
            "@drake//tools/cc_toolchain:apple": [],
        }),
        linkopts = select({
            "@drake//tools/cc_toolchain:linux": [
                "-Wl,--exclude-libs=lib{}.a".format(static_archive_name),
                "-Wl,--exclude-libs=lib{}.pic.a".format(static_archive_name),
            ],
            "@drake//tools/cc_toolchain:apple": [],
        }),
        linkstatic = True,
        visibility = visibility,
    )
