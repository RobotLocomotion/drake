# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.symlink(
            Label("@drake//tools/workspace/opencl:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )
    elif os_result.is_ubuntu:
        hdrs = [
            "CL/cl.h",
            "CL/opencl.h",
        ]
        for hdr in hdrs:
            repository_ctx.symlink(
                "/usr/include/{}".format(hdr),
                "include/{}".format(hdr),
            )
        repository_ctx.symlink(
            Label("@drake//tools/workspace/opencl:package-ubuntu.BUILD.bazel"),
            "BUILD.bazel",
        )
    else:
        fail("Operating system is NOT supported {}".format(os_result))

opencl_repository = repository_rule(
    local = True,
    implementation = _impl,
)
