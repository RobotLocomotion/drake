# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "82bd3a223e3227c70832307e53a65c13c1e5f81b",
        sha256 = "4a50e23b6715d625fdd1f2d5f23152bab8af87ff11681b58aa14d7d61bb3280e",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
