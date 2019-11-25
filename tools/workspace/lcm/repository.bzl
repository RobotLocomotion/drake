# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        commit = "fd9600a573a4feac7607216c00866d5824e9d1e2",
        sha256 = "b2f512b473a852f82970899b6dcc2580f17df933fd16b3ca3560db18060effac",  # noqa
        build_file = "@drake//tools/workspace/lcm:package.BUILD.bazel",
        mirrors = mirrors,
    )
