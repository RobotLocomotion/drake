# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def picosha2_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "okdshin/PicoSHA2",
        commit = "27fcf6979298949e8a462e16d09a0351c18fcaf2",
        sha256 = "18d82bb79c021ccf4ce58125b64691accef54237ba5194462740bacf8b39d8a9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
