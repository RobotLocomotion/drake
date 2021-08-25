# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        build_flavor = "macos"
        repository_ctx.symlink(
            "/usr/local/opt/yaml-cpp/include/yaml-cpp",
            "include/yaml-cpp",
        )
    elif os_result.is_ubuntu:
        build_flavor = "ubuntu-{}".format(os_result.ubuntu_release)
        repository_ctx.symlink("/usr/include/yaml-cpp", "include/yaml-cpp")
    else:
        fail("Operating system is NOT supported {}".format(os_result))

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
