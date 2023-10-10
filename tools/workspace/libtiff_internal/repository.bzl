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
        commit = "v4.6.0",
        sha256 = "c760de148e0b2ceec115dd5afa660e91617aac4a94da997aa8a8fc2a58f1d378",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
