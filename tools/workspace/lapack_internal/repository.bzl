load("//tools/workspace:github.bzl", "github_archive")

def lapack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Reference-LAPACK/lapack",
        upgrade_advice = """
        If the upstream list of source files changes, then the linter will fail
        and you'll need to follow its advice to copy the new lock file.
        """,
        commit = "v3.12.1",
        sha256 = "2ca6407a001a474d4d4d35f3a61550156050c48016d949f0da0529c0aa052422",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
