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
    elif os_result.ubuntu_release == "18.04":
        # On Ubuntu 18.04 (Bionic), download and extract Drake's pre-compiled
        # Doxygen binary.
        url = [
            pattern.format(archive = "doxygen-1.8.17-bionic-x86_64.tar.gz")
            for pattern in repo_ctx.attr.mirrors.get("doxygen")
        ]
        repo_ctx.download_and_extract(
            url = url,
            sha256 = "daea2ba8c57056801e0ce1e9387d5e4bf148e6898ab237f1972392b860162db6",  # noqa
            type = "tar.gz",
        )
    elif os_result.ubuntu_release == "20.04":
        # On Ubuntu 20.04 (Focal), just symlink Doxygen from the Ubuntu package
        # archive.
        repo_ctx.symlink("/usr/bin/doxygen", "doxygen")
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
