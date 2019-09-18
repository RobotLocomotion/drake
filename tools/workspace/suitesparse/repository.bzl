# -*- mode: python -*-

load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

def _impl(repo_ctx):
    # Find the include path.
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_ubuntu:
        include = "/usr/include/suitesparse"
    elif os_result.is_mac:
        include = "/usr/local/include/suitesparse"
    else:
        fail("Unknown OS")

    # Grab the relevant headers.
    hdrs = [
        "SuiteSparse_config.h",
        "amd.h",
        "ldl.h",
    ]
    for hdr in hdrs:
        repo_ctx.symlink(include + "/" + hdr, "include/" + hdr)

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/suitesparse:package.BUILD.bazel"),
        "BUILD.bazel",
    )

suitesparse_repository = repository_rule(
    local = True,
    implementation = _impl,
)
