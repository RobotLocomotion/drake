# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        libdir = "{}/opt/yaml-cpp/lib".format(os_result.homebrew_prefix)
        build_flavor = "generic"
        repository_ctx.symlink(
            "{}/opt/yaml-cpp/include/yaml-cpp".format(
                os_result.homebrew_prefix,
            ),
            "include/yaml-cpp",
        )
    elif os_result.is_ubuntu:
        libdir = "/usr/lib/x86_64-linux-gnu"
        if os_result.ubuntu_release == "18.04":
            build_flavor = "ubuntu-{}".format(os_result.ubuntu_release)
        else:
            build_flavor = "generic"
        repository_ctx.symlink("/usr/include/yaml-cpp", "include/yaml-cpp")
    elif os_result.is_manylinux:
        libdir = "/opt/drake-dependencies/lib"
        build_flavor = "generic"
        repository_ctx.symlink(
            # TODO(jwnimmer-tri) Ideally, we wouldn't be hard-coding paths when
            # using manylinux.
            "/opt/drake-dependencies/include/yaml-cpp",
            "include/yaml-cpp",
        )
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    # Declare the libdir.
    repository_ctx.file(
        "vars.bzl",
        content = "LIBDIR = \"{}\"\n".format(libdir),
        executable = False,
    )

    # Add the BUILD file.
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
