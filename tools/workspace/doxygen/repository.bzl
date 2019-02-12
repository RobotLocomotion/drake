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

    # On macOS, just symlink homebrew Doxygen.
    if os_result.is_macos:
        repo_ctx.symlink("/usr/local/bin/doxygen", "doxygen")
        return

    # On Ubuntu, download Drake's pre-compiled Doxygen binary.
    version = "1.8.15"
    archive = "doxygen-{version}-{codename}-x86_64.tar.gz".format(
        version = version,
        codename = {
            "16.04": "xenial",
            "18.04": "bionic",
        }[os_result.ubuntu_release],
    )
    url = [
        pattern.format(archive = archive)
        for pattern in repo_ctx.attr.mirrors.get("doxygen")
    ]
    sha256 = {
        "16.04": "7b0a11bab077b8aa677aad09ca89d978ef2e75b56742a4b6870d064c8bd6d4bc",  # noqa
        "18.04": "16b4ce1fcee27495c0de23dc4a2ab9bd24ee218800a2fb0db17a9c5bf8955e4e",  # noqa
    }[os_result.ubuntu_release]
    repo_ctx.download_and_extract(
        url = url,
        sha256 = sha256,
    )

doxygen_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
"""Provides a library target for @doxygen//:doxygen.  On macOS, uses homebrew;
on Ubuntu, downloads and extracts a precompiled binary release of Doxygen.
"""
