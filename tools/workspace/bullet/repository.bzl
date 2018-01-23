# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bullet_repository(name):
    github_archive(
        name = name,
        repository = "bulletphysics/bullet3",
        commit = "2.86.1",
        sha256 = "c058b2e4321ba6adaa656976c1a138c07b18fc03b29f5b82880d5d8228fbf059",  # noqa
        build_file = "@drake//tools/workspace/bullet:package.BUILD.bazel",
    )
