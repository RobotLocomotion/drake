# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def styleguide_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/styleguide",
        commit = "a61ad696eabc64ec06eb965b4925fdff7e433a36",
        sha256 = "18e5ddd60b54fd7181bcf3c917bc6fd7166faed097e71ca16463e87d5e7b7cc4",  # noqa
        build_file = "@drake//tools/workspace/styleguide:package.BUILD.bazel",
        mirrors = mirrors,
    )
