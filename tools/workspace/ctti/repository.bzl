# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def ctti_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "Manu343726/ctti",
        commit = "d7e9828b82ce7a6321465fbd84f9bccb772c7f43",
        sha256 = "baee000068d1b691d935084183296f488f4455c102da2a40414d536ab124edba",  # noqa
        build_file = "@drake//tools/workspace/ctti:package.BUILD.bazel",
        mirrors = mirrors,
    )
