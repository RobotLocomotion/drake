# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "15afd9aa83a9f25964add3d45f57aed7c2484142",
        sha256 = "b0451c92c453ca7844a26516eb2fb27beebd86e2afb31c4a2b5e2487d2fd1d6c",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
