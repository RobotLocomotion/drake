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
        commit = "v4.7.0",
        sha256 = "8f568a0dfac2e514074b04d7368c22e8afc1009af29780762a2536fd4d111e16",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
