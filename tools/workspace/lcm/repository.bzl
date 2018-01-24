# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(name):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "87866bd0dbb1f9d5a0f662a6f5caecf469fd42d2",
        sha256 = "fd0afaf29954c26a725626b7bd24e873e303e84bb62dfcc05162be3f5ae30cd1",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
    )
