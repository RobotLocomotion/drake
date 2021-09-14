# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        # On macOS, no targets should depend on @glx.
        build_flavor = "macos"
    elif os_result.is_ubuntu or os_result.is_manylinux:
        build_flavor = "ubuntu"
        hdrs = [
            "GL/glx.h",
            "GL/glxext.h",
        ]
        for hdr in hdrs:
            repository_ctx.symlink(
                "/usr/include/{}".format(hdr),
                "include/{}".format(hdr),
            )
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    repository_ctx.symlink(
        Label(
            "@drake//tools/workspace/glx:package-{}.BUILD.bazel".format(
                build_flavor,
            ),
        ),
        "BUILD.bazel",
    )

glx_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
