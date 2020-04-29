# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        # Working temporary SHA1 until the models PR 9 is merged.
        commit = "0dad1ff4dc6ba56f29d34caa2a566f62316007ef",
        sha256 = "d5f7f4d4a7be5e7e808bad8f22a88ca96f91a3c5750cd529d59c674535623d20",  # noqa
        build_file = "@drake//tools/workspace/models:package.BUILD.bazel",
        mirrors = mirrors,
    )
