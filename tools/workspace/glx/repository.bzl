# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        # On macOS, no targets should depend on @glx.
        repository_ctx.symlink(
            Label("@drake//tools/workspace/glx:package-macos.BUILD.bazel"),
            "BUILD.bazel",
        )
    elif os_result.is_ubuntu:
        hdrs = [
            "GL/glx.h",
            "GL/glxext.h",
        ]
        for hdr in hdrs:
            repository_ctx.symlink(
                "/usr/include/{}".format(hdr),
                "include/{}".format(hdr),
            )
        repository_ctx.symlink(
            Label("@drake//tools/workspace/glx:package-ubuntu.BUILD.bazel"),
            "BUILD.bazel",
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

glx_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
