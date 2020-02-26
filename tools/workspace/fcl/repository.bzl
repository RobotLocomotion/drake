# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "97455a46de121fb7c0f749e21a58b1b54cd2c6be",
        sha256 = "9020207d167ae3ffd6c11e483406dd0aeb31634ac1125d9dd88962d90bcb5f57",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
