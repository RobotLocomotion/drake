# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def nanoflann_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "jlblancoc/nanoflann",
        commit = "v1.4.3",
        sha256 = "cbcecf22bec528a8673a113ee9b0e134f91f1f96be57e913fa1f74e98e4449fa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
