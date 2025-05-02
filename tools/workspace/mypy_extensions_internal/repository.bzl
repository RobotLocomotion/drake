load("//tools/workspace:github.bzl", "github_archive")

def mypy_extensions_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        # This dependency is part of a "cohort" defined in
        # drake/tools/workspace/new_release.py.  When practical, all members
        # of this cohort should be updated at the same time.
        repository = "python/mypy_extensions",
        commit = "1.1.0",
        sha256 = "178030dd39335042c2c5becccc596c2f307f796868f9c627da3fe14d76de9d97",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
