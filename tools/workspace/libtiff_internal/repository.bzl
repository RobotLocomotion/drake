load("//tools/workspace:github.bzl", "github_archive")

def libtiff_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # N.B. Upstream is https://gitlab.com/libtiff/libtiff but this github
        # mirror seems to be kept up to date.
        repository = "libsdl-org/libtiff",
        upgrade_advice = """
        The package.BUILD.bazel file hard-codes the version number and release
        date; be sure to update those to match the new commit.
        """,
        upgrade_type = "tag",
        commit = "v4.7.2",
        sha256 = "dd030ae5d6483033bd88d840fc97e36adc3022b844d9d896a3b5a50f2d660d6f",  # noqa
        build_file = ":package.BUILD.bazel",
        patch_cmds = [
            # On a macOS case-insensitive filesystem, this conflicts with
            # `#include <version>` from the standard library.
            "rm -f VERSION",
        ],
        mirrors = mirrors,
    )
