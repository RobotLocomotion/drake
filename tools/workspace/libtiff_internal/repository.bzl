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
        commit = "v4.7.1",
        sha256 = "a3faec056a490c62b8749847317521204f8c4d9933cce834e691a74524ec38fe",  # noqa
        build_file = ":package.BUILD.bazel",
        patch_cmds = [
            # On a macOS case-insensitive filesystem, this conflicts with
            # `#include <version>` from the standard library.
            "rm -f VERSION",
        ],
        mirrors = mirrors,
    )
