# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        build_flavor = "macos"
        repository_ctx.symlink(
            "/usr/local/opt/double-conversion/include",
            "include",
        )
    elif os_result.is_ubuntu:
        build_flavor = "ubuntu"
        repository_ctx.symlink(
            "/usr/include/double-conversion",
            "include/double-conversion",
        )
    elif os_result.is_manylinux:
        build_flavor = "ubuntu"
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    repository_ctx.symlink(
        Label(
            "@drake//tools/workspace/double_conversion:" +
            "package-{}.BUILD.bazel".format(
                build_flavor,
            ),
        ),
        "BUILD.bazel",
    )

double_conversion_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
