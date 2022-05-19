# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos or os_result.is_macos_wheel:
        build_flavor = "macos"
        repository_ctx.symlink(
            "{}/opt/nlopt/include/nlopt.h".format(os_result.homebrew_prefix),
            "include/nlopt.h",
        )
        repository_ctx.symlink(
            "{}/opt/nlopt/include/nlopt.hpp".format(os_result.homebrew_prefix),
            "include/nlopt.hpp",
        )
    elif os_result.is_ubuntu or os_result.is_manylinux:
        build_flavor = "ubuntu-{}".format(os_result.ubuntu_release)
        repository_ctx.symlink("/usr/include/nlopt.h", "include/nlopt.h")
        repository_ctx.symlink("/usr/include/nlopt.hpp", "include/nlopt.hpp")
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    repository_ctx.symlink(
        Label(
            "@drake//tools/workspace/nlopt:package-{}.BUILD.bazel".format(
                build_flavor,
            ),
        ),
        "BUILD.bazel",
    )

nlopt_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
