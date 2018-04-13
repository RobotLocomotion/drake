# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ccd_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "danfis/libccd",
        commit = "63d3a911f016465a2ecf169d0c8bff8b601f1715",
        sha256 = "1032cae04202330c5bcc9a652d75bef64669656bf4ea6cab74c5ff11c3f7a301",  # noqa
        build_file = "@drake//tools/workspace/ccd:package.BUILD.bazel",
        mirrors = mirrors,
    )
