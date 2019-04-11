# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "252444cad5b140331a68bd07488289b558080106",
        sha256 = "3a5707dc8e5a2e71e16c24d899b54bf10888fe781a7253a363a8eada66c2cb68",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
