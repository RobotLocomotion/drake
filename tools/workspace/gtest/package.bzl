# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def gtest_repository(name):
    github_archive(
        name = name,
        repository = "google/googletest",
        commit = "release-1.8.0",
        sha256 = "58a6f4277ca2bc8565222b3bbd58a177609e9c488e8a72649359ba51450db7d8",  # noqa
        build_file = "@drake//tools/workspace/gtest:package.BUILD.bazel",
    )
