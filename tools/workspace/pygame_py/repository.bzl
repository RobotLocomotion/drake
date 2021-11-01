# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")
load("@drake//tools/workspace:pypi_wheel.bzl", "setup_pypi_wheel")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    # On Ubuntu 18.04, python3-pygame is not available so we need to use a
    # wheel.
    if os_result.is_ubuntu and os_result.ubuntu_release == "18.04":
        result = setup_pypi_wheel(
            repository_ctx,
            package = "pygame",
            version = "1.9.6",
            version_pin = True,  # It's not worth upgrading to 2.0.
            pypi_tag = "cp36-cp36m-manylinux1_x86_64",
            blake2_256 = "8e24ede6428359f913ed9cd1643dd5533aefeb5a2699cc95bea089de50ead586",  # noqa
            sha256 = "c895cf9c1b6d1cbba8cb8cc3f5427febcf8aa41a9333697741abeea1c537a350",  # noqa
            mirrors = repository_ctx.attr.mirrors,
        )
        if result.error != None:
            fail("Unable to complete setup for @{} repository: {}".format(
                # (forced line break)
                repository_ctx.name,
                result.error,
            ))
        return

    # On newer Ubuntu, we use python3-pygame from apt.  On macOS, we do not
    # offer a working @pygame repostory.  Either way, the BUILD is the same.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/pygame_py:package-stub.BUILD.bazel"),
        "BUILD.bazel",
    )

pygame_py_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(
            doc = """Dictionary such as the provided DEFAULT_MIRRORS from
            @drake//tools/workspace:mirrors.bzl""",
            mandatory = True,
            allow_empty = False,
        ),
    },
    implementation = _impl,
)
