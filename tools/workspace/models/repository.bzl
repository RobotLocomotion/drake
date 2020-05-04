# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        # DNM Switch back to master.
        commit = "8a21b33e4fab3a764f97b5677a93682197de5c8e",
        sha256 = "67efe784b5fb08ad3cf0f645231ed40ecc1bc64c5b01a78dbdf39b564de653c4",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
