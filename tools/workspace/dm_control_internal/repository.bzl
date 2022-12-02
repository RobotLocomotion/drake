# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "e3d4c0aecfd710cd7428eb77af203dc5fbdacc0f",
        sha256 = "2617ac8b1b6566f63508a1cb2c90091bf7b763e94d5a78603e0f002dde10495d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
