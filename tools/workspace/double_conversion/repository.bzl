load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        libdir = "{}/opt/double-conversion/lib".format(
            os_result.homebrew_prefix,
        )
        repository_ctx.symlink(
            "{}/opt/double-conversion/include".format(
                os_result.homebrew_prefix,
            ),
            "include",
        )
    elif os_result.is_ubuntu:
        libdir = "/usr/lib/x86_64-linux-gnu"
        repository_ctx.symlink(
            "/usr/include/double-conversion",
            "include/double-conversion",
        )
    elif os_result.is_manylinux or os_result.is_macos_wheel:
        libdir = "/opt/drake-dependencies/lib"
        repository_ctx.symlink(
            "/opt/drake-dependencies/include/double-conversion",
            "include/double-conversion",
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
        Label("@drake//tools/workspace/double_conversion:package.BUILD.bazel"),
        "BUILD.bazel",
    )

double_conversion_repository = repository_rule(
    local = True,
    configure = True,
    implementation = _impl,
)
