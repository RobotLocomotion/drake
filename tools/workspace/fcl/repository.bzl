# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "1271b65c25b2595c3232c29ffe06d465f4518aa2",
        sha256 = "6c0392c579b88920a2ebfa83cc58bcb34c27f9e26ff9598ee7f0da85150d1323",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
