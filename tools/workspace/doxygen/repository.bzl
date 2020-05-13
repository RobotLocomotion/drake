# -*- mode: python -*-

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load(
    "@drake//tools/workspace:os.bzl",
    "determine_os",
)

def _impl(repo_ctx):
    os_result = determine_os(repo_ctx)
    if os_result.error != None:
        fail(os_result.error)

    # Add the BUILD file -- same for both platforms.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/doxygen:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    if os_result.is_macos:
        # On macOS, just symlink Doxygen from Homebrew.
        repo_ctx.symlink("/usr/local/bin/doxygen", "doxygen")
    elif os_result.is_ubuntu:
        # On Ubuntu, download and extract Drake's pre-compiled Doxygen binary.
        archive = "doxygen-1.8.16-{codename}-x86_64.tar.gz".format(
            codename = {
                "18.04": "bionic",
                "20.04": "focal",
            }[os_result.ubuntu_release],
        )
        url = [
            pattern.format(archive = archive)
            for pattern in repo_ctx.attr.mirrors.get("doxygen")
        ]
        sha256 = {
            "18.04": "71323bac4f455b2ba8184f3c3c8a38033e713c030c3f547874beb7b8000e65f6",  # noqa
            "20.04": "e5c3fec1daf7ec70f33c6e42561c2b6642eaf8cd3f758bf488c2dc651f2046d3",  # noqa
        }[os_result.ubuntu_release]
        repo_ctx.download_and_extract(
            url = url,
            sha256 = sha256,
            type = "tar.gz",
        )
    else:
        fail("Operating system is NOT supported", attr = os_result)

doxygen_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
"""Provides a library target for @doxygen//:doxygen.  On macOS, uses homebrew;
on Ubuntu, downloads and extracts a precompiled binary release of Doxygen.
"""
