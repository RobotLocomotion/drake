# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def qhull_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "qhull/qhull",
        commit = "2020.2",
        sha256 = "59356b229b768e6e2b09a701448bfa222c37b797a84f87f864f97462d8dbc7c5",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/cxx20.patch",
            ":patches/disable_dead_code.patch",
            ":patches/vendor_cxx.patch",
        ],
        mirrors = mirrors,
    )
