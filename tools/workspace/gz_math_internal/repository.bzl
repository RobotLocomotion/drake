# -*- python -*-

load("//tools/workspace:github.bzl", "github_archive")

def gz_math_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "gazebosim/gz-math",
        commit = "gz-math7_7.0.0-pre1",
        sha256 = "5155c89930d436442774b49c1ad364d4f8465696939ceed13de5f973e095cc10",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
