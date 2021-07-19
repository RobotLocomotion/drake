# -*- mode: python; -*-
# vi: set ft=python:

load("@drake//tools/workspace:os.bzl", "determine_os")
load(
    "@drake//tools/workspace:pypi_wheel.bzl",
    "download_and_extract_pypi_wheel",
)

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)

    if os_result.error != None:
        fail(os_result.error)

    PYPI_WHEEL_PACKAGE = "castxml"
    PYPI_WHEEL_VERSION = "0.4.3"

    if os_result.is_macos:
        download_and_extract_pypi_wheel(
            repository_ctx,
            package = PYPI_WHEEL_PACKAGE,
            version = PYPI_WHEEL_VERSION,
            pypi_tag = "py3-none-macosx_10_13_x86_64",
            blake2_256 = "a69b1a2d9b75520c169ec64e331a1d40ad3f79c03d2fdb3da7c5f4a9ea462575",  # noqa
            sha256 = "51f986eeff90fcbf4b52b65100ac300f77aaaabd61a17c5ca24f8e8aae812cc4",  # noqa
            mirrors = repository_ctx.attr.mirrors,
        )
    elif os_result.is_ubuntu:
        download_and_extract_pypi_wheel(
            repository_ctx,
            package = PYPI_WHEEL_PACKAGE,
            version = PYPI_WHEEL_VERSION,
            pypi_tag = "py3-none-manylinux2014_x86_64",
            blake2_256 = "5701896fa90f3995672c911a46a511a4c7b21a51f1269e19db81c891f38409e4",  # noqa
            sha256 = "6b40258aca3b81f9af223586984933ef63d36ea36ff307e36aea374d21a19e4f",  # noqa
            mirrors = repository_ctx.attr.mirrors,
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

    repository_ctx.symlink(
        Label("@drake//tools/workspace/castxml:package.BUILD.bazel"),
        "BUILD.bazel",
    )

castxml_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
