# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "2112037d3f0490f83c9e2d237886eb113b7b0d31",
        sha256 = "03bc5fb6adb151de47cec044af52bea4c0961ee3b15a4030caeb78a7acc0ed5b",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
