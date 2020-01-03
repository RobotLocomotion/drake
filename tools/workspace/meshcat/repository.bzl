# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        commit = "2b516ed00b36bf71c00e000c5be73c840fb97333",
        sha256 = "87b4f285a05a7dde34fc5b1ee58fd046b15524972d7fdc298d581382f668e78f",  # noqa
        build_file = "@drake//tools/workspace/meshcat:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
