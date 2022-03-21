# -*- python -*-

# This file contains a linter rule that ensures that only our allowed set of
# third-party dependencies are used as "interface deps". In almost all cases,
# we should be using "implementation deps" when using third-party libraries.
# Refer to drake_cc_library documentation for details.

# Drake's allowed list of third-party libraries. Do not add new entries here
# without consulting Drake's build system maintainers (see #7451). Keep this
# list in sync with test/header_dependency_test.py.
_ALLOWED_EXTERNALS = [
    "eigen",
    "fmt",
    "lcm",
    "optitrack",
    "spdlog",

    # The entries that follow are defects; we should work to remove them.
    "ccd",
    "cds",
    "clp",
    "conex",
    "csdp",
    "double_conversion",
    "dreal",
    "fcl",
    "ghc_filesystem",
    "glew",
    "glib",
    "glx",
    "gurobi",
    "ibex",
    "ignition_math",
    "ignition_utils",
    "ipopt",
    "lcm",
    "libjpeg",
    "liblz4",
    "liblzma",
    "libpng",
    "libtiff",
    "mosek",
    "nlopt",
    "openblas",
    "opengl",
    "osqp",
    "picosat",
    "qdldl",
    "qhull",
    "scs",
    "sdformat",
    "snopt",
    "suitesparse",
    "tinyobjloader",
    "tinyxml2",
    "vtk",
    "zlib",
]

# Drake's allowed list of public preprocessor definitions. The only things
# permitted here are definitions required by the _ALLOWED_EXTERNALS, above.
_ALLOWED_DEFINES = [
    "EIGEN_MPL2_ONLY",
    "FMT_HEADER_ONLY=1",
    "FMT_NO_FMT_STRING_ALIAS=1",
    "HAVE_SPDLOG",
    "SPDLOG_COMPILED_LIB",
    "SPDLOG_FMT_EXTERNAL",
    "SPDLOG_SHARED_LIB",

    # The entries that follow are defects; we should work to remove them.
    "CCD_STATIC_DEFINE",
    "FCL_STATIC_DEFINE",
    "HAVE_CSTDDEF",
    "SDFORMAT_DISABLE_CONSOLE_LOGFILE",
    "SDFORMAT_STATIC_DEFINE",
    "USE_LAPACK=1",
    "_DARWIN_C_SOURCE",
]

def _cc_check_allowed_headers_impl(ctx):
    details = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    ).compilation_context
    defines = depset(transitive = [
        details.defines,
        details.local_defines,
    ])
    headers = depset(direct = (
        details.direct_headers +
        details.direct_private_headers +
        details.direct_public_headers +
        details.direct_textual_headers
    ), transitive = [
        details.headers,
    ])
    failures = []
    for item in defines.to_list():
        if item not in _ALLOWED_DEFINES:
            failures.append("-D{}".format(item))
    for item in headers.to_list():
        path = item.path
        if path.startswith("external/"):
            repo = path.split("/")[1]
            if repo not in _ALLOWED_EXTERNALS:
                failures.append("@{}".format(repo))
    if failures:
        fail("\n".join([
            "Dependency pollution has leaked into Drake's public headers:",
        ] + [
            "{} is not allowed in interface_deps".format(item)
            for item in depset(failures).to_list()
        ]))

cc_check_allowed_headers = rule(
    implementation = _cc_check_allowed_headers_impl,
    doc = """
Ensures that only our allowed set of third-party dependencies are used as
"interface deps". In almost all cases, we should be using "implementation deps"
when using third-party libraries.  Refer to drake_cc_library documentation for
details.
""",
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
    },
    fragments = ["cpp"],
)
