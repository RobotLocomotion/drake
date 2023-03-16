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
        error_messages = [
            "Dependency pollution has leaked into Drake's public headers:",
        ] + [
            " {} is not allowed in interface_deps".format(item)
            for item in depset(failures).to_list()
        ] + [
            "To resolve this problem, alter your drake_cc_library to either:",
            " split up 'interface_deps = [...]' vs 'deps = [...]', or",
            " use 'internal = True'.",
            "Check the drake_cc_library documentation for details, or",
            " ask for help on drakedevelopers#build slack.",
        ]
        content = "echo 'ERROR: The header_lint test has failed!'\n"
        content += "cat <<EOF\n" + "\n".join(error_messages) + "\nEOF\n"
        content += "exit 1\n"
    else:
        content = "echo 'PASS: The header_lint test has passed.'\n"
    ctx.actions.write(output = ctx.outputs.sh_src, content = content)

_generate_error_messages = rule(
    implementation = _cc_check_allowed_headers_impl,
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
        "sh_src": attr.output(mandatory = True),
    },
    fragments = ["cpp"],
)

def cc_check_allowed_headers(name, deps = []):
    """Ensures that only an allowed set of third-party dependencies are used as
    'interface deps'. In almost all cases, we should be using 'implementation
    deps' when using third-party libraries.  Refer to drake_cc_library docs for
    details.
    """
    sh_src = name + ".sh"
    _generate_error_messages(
        name = name + ".genrule",
        sh_src = sh_src,
        testonly = True,
        tags = ["manual"],
        deps = deps,
    )
    native.sh_test(
        name = name,
        tags = ["lint"],
        srcs = [":" + sh_src],
    )
