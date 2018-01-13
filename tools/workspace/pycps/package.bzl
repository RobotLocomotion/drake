# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def pycps_repository(name):
    github_archive(
        name = name,
        repository = "mwoehlke/pycps",
        commit = "544c1ded81b926a05b3dedb06504bd17bc8d0a95",
        sha256 = "0b97cbaae107e5ddbe89073b6e42b679130f1eb81b913aa93da9e72e032a137b",  # noqa
        build_file = "@drake//tools/workspace/pycps:package.BUILD.bazel",
    )
