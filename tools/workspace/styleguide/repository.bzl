# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "7c296218c7cb6aa66aeb0726c45b3bb9fa95dd03",
        sha256 = "dbe53dc6d74204550ab46c662050c9ef96101b3c94db48b011e9f2ea569450aa",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
