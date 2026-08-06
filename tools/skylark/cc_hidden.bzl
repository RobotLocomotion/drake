load("@with_cfg.bzl", "with_cfg")
load("//tools/skylark:cc.bzl", "cc_library")

_builder = with_cfg(cc_library)
_builder.extend("conlyopt", ["-fvisibility=hidden"])
_builder.extend("features", ["-supports_dynamic_linker"])
_cc_static_hidden_library, _ = _builder.build()

def cc_static_hidden_library(
    name,
    *,
    deps,
    visibility = ["//visibility:private"]):
    """Recompiles the libraries listed in `deps` adding
    `conlyopts = ["-fvisibilty=hidden"]` and using only static linking.

    This is useful when linking third-party C/C++ libraries into libdrake.so
    when we don't control the BUILD files (so can't directly set copts or
    linkstatic).
    """

    if not deps:
        fail("cc_static_hidden_library is intended to wrap another library as listed in `deps`.")

    # Add upstream build flags for individual third-party libraries. These are
    # set here as the one source of truth (as opposed to each external's
    # callsite) so that if drake's third-party dependencies have transitive
    # dependencies on each other, only one copy is built using the same flags.
    defines = [
        # Use a drake_vendor hidden namespace instead of nlohmann_json's
        # default ABI namespace based on its version number. This prevents ODR
        # violations in case downstream code also links to nlohmann_json.
        "'NLOHMANN_JSON_NAMESPACE_BEGIN=namespace nlohmann { inline namespace drake_vendor __attribute__((visibility(\"hidden\"))) {'",
    ]

    _cc_static_hidden_library(
        name = name,
        defines = defines,
        deps = deps,
        visibility = visibility,
    )

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
