# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "98893d8cb34cc86203235f2e41b740bf216b97c3",
        sha256 = "5c9beb7e4d6f2aef4f6a51dc9c7ee687ba3a2c9a25f8228d9a0b126980db4477",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
