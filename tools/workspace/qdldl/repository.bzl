# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def qdldl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "oxfordcontrol/qdldl",
        # When updating this commit, see drake/tools/workspace/qdldl/README.md.
        commit = "v0.1.4",
        sha256 = "4eaed3b2d66d051cea0a57b0f80a81fc04ec72c8a906f8020b2b07e31d3b549c",  # noqa
        build_file = "@drake//tools/workspace/qdldl:package.BUILD.bazel",
        mirrors = mirrors,
    )
