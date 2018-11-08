# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "3f59638077de81097336b300959dedcb60538d99",
        sha256 = "5b2d017920c5563b739b381a7721d375cfb6463a079af7fe5ccd8dbb3aeafff7",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
