# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ccd_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "danfis/libccd",
        commit = "7931e764a19ef6b21b443376c699bbc9c6d4fba8",
        sha256 = "479994a86d32e2effcaad64204142000ee6b6b291fd1859ac6710aee8d00a482",  # noqa
        build_file = "@drake//tools/workspace/ccd:package.BUILD.bazel",
        mirrors = mirrors,
    )
