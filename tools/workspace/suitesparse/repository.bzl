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
        lib = "/usr/lib"
    elif os_result.is_macos:
        include = "/usr/local/opt/suite-sparse/include"
        lib = "/usr/local/opt/suite-sparse/lib"
    elif os_result.is_manylinux:
        # TODO(jwnimmer-tri) Ideally, we wouldn't be hard-coding paths when
        # using manylinux.
        include = "/opt/drake-dependencies/include"
        lib = "/opt/drake-dependencies/lib"
    else:
        fail("Unknown OS")

    # Grab the relevant headers.
    hdrs = [
        "SuiteSparse_config.h",
        "amd.h",
    ]
    for hdr in hdrs:
        repo_ctx.symlink(include + "/" + hdr, "include/" + hdr)

    # Declare the libdir.
    repo_ctx.file(
        "vars.bzl",
        content = "LIBDIR = \"{}\"\n".format(lib),
        executable = False,
    )

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/suitesparse:package.BUILD.bazel"),
        "BUILD.bazel",
    )

suitesparse_repository = repository_rule(
    local = True,
    implementation = _impl,
)
