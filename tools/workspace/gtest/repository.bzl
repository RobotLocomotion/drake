# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "release-1.11.0",
        sha256 = "b4870bf121ff7795ba20d20bcdd8627b8e088f2d1dab299a031c1034eddc93d5",  # noqa
        build_file = "@drake//tools/workspace/gtest:package.BUILD.bazel",
        patches = [
            "@drake//tools/workspace/gtest:dont-define-test-f.patch",
        ],
        mirrors = mirrors,
    )
