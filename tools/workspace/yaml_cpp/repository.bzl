# -*- python -*-

# TODO(jwnimmer-tri) Remove this entire directory on or after 2022-07-01, as
# well as removing yaml from our setup text files:
# setup/mac/binary_distribution/Brewfile
# setup/ubuntu/binary_distribution/packages-*.txt

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        build_flavor = "macos"
        repository_ctx.symlink(
            "{}/opt/yaml-cpp/include/yaml-cpp".format(
                os_result.homebrew_prefix,
            ),
            "include/yaml-cpp",
        )
    elif os_result.is_ubuntu:
        build_flavor = "ubuntu-{}".format(os_result.ubuntu_release)
        repository_ctx.symlink("/usr/include/yaml-cpp", "include/yaml-cpp")
    else:
        # Since @yaml_cpp is deprecated, we can just soft-fail here.
        build_flavor = "ubuntu-20.04"
        print("Operating system is NOT supported {}".format(os_result))

    repository_ctx.symlink(
        Label(
            "@drake//tools/workspace/yaml_cpp:package-{}.BUILD.bazel".format(
                build_flavor,
            ),
        ),
        "BUILD.bazel",
    )

yaml_cpp_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
