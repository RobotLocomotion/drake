# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        repository_ctx.symlink(
            "/usr/local/opt/nlopt/include/nlopt.h",
            "include/nlopt.h",
        )
        repository_ctx.symlink(
            "/usr/local/opt/nlopt/include/nlopt.hpp",
            "include/nlopt.hpp",
        )
        repository_ctx.symlink(
            Label(
                "@drake//tools/workspace/nlopt:package-macos.BUILD.bazel",
            ),
            "BUILD.bazel",
        )
    elif os_result.is_ubuntu:
        repository_ctx.symlink("/usr/include/nlopt.h", "include/nlopt.h")
        repository_ctx.symlink("/usr/include/nlopt.hpp", "include/nlopt.hpp")
        repository_ctx.symlink(
            Label(
                ("@drake//tools/workspace/nlopt" +
                 ":package-ubuntu-{}.BUILD.bazel").format(
                    os_result.ubuntu_release,
                ),
            ),
            "BUILD.bazel",
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

nlopt_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
