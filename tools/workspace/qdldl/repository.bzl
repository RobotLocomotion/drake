# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def qdldl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/qdldl",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v0.1.5",
        sha256 = "2868b0e61b7424174e9adef3cb87478329f8ab2075211ef28fe477f29e0e5c99",  # noqa
        build_file = "@drake//tools/workspace/qdldl:package.BUILD.bazel",
        mirrors = mirrors,
    )
