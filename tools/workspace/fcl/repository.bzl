# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def fcl_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "flexible-collision-library/fcl",
        commit = "09f846c67a438c6ddbd8ec2f96b35547bcc06e2a",
        sha256 = "7ba29e390ad7b8f2e290f1721726dfdbbdadaae70bf300de6a8d35b99ec58847",  # noqa
        build_file = "@drake//tools/workspace/fcl:package.BUILD.bazel",
        mirrors = mirrors,
    )
