# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.symlink(
            "/usr/local/opt/double-conversion/include",
            "include",
        )
        repository_ctx.symlink(
            Label(
                "@drake//tools/workspace/double_conversion:package-macos.BUILD.bazel",  # noqa
            ),
            "BUILD.bazel",
        )
    elif os_result.is_ubuntu:
        repository_ctx.symlink(
            "/usr/include/double-conversion",
            "include/double-conversion",
        )
        repository_ctx.symlink(
            Label(
                "@drake//tools/workspace/double_conversion:package-ubuntu.BUILD.bazel",  # noqa
            ),
            "BUILD.bazel",
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

double_conversion_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
