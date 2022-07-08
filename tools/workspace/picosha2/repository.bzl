# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def picosha2_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "okdshin/PicoSHA2",
        commit = "1677374f23352716fc52183255a40c1b8e1d53eb",
        sha256 = "82f69e96c4ce2ba07eea2915d9300ad5d1a2303edb5a323c5a3a16bf18f484f4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
