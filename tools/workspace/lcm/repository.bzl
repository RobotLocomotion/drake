# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "55abc1f31b14a4e545562607d3d40c7ad1410a0f",
        sha256 = "78e9d449f46afcf6fe3a7f0fba4b9a0f539e5bf6cc2f463f26acb98a5319bef4",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
