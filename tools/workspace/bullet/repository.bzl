# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def bullet_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "bulletphysics/bullet3",
        commit = "2.89",
        sha256 = "621b36e91c0371933f3c2156db22c083383164881d2a6b84636759dc4cbb0bb8",  # noqa
        build_file = "@drake//tools/workspace/bullet:package.BUILD.bazel",
        mirrors = mirrors,
    )
