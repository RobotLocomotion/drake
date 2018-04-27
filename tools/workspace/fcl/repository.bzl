# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "2d3d8f7ea6f255b5f3743658c49ad869b9978c03",
        sha256 = "8a251eee79e916abe34d9f091ee247e77b5246fbe75ced544e9d2e4b460661ea",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
