# -*- mode: python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        build_flavor = "macos"
        repository_ctx.symlink(
            "/usr/local/opt/nlopt/include/nlopt.h",
            "include/nlopt.h",
        )
        repository_ctx.symlink(
            "/usr/local/opt/nlopt/include/nlopt.hpp",
            "include/nlopt.hpp",
        )
    elif os_result.is_ubuntu:
        build_flavor = "ubuntu-{}".format(os_result.ubuntu_release)
        repository_ctx.symlink("/usr/include/nlopt.h", "include/nlopt.h")
        repository_ctx.symlink("/usr/include/nlopt.hpp", "include/nlopt.hpp")
    elif os_result.is_manylinux:
        # Although we expect that "manylinux" is based on Ubuntu 18.04, the
        # "flavor" here relates to what spelling of the linkopts we want to be
        # using.  As drake-dependencies is currently formulated, the linkopts
        # from the ubuntu-18.04.BUILD.bazel are satisfactory.
        build_flavor = "ubuntu-18.04"
        repository_ctx.symlink(
            # TODO(jwnimmer-tri) Ideally, we wouldn't be hard-coding paths when
            # using manylinux.
            "/opt/drake-dependencies/include/nlopt.h",
            "include/nlopt.h",
        )
        repository_ctx.symlink(
            "/opt/drake-dependencies/include/nlopt.hpp",
            "include/nlopt.hpp",
        )
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
