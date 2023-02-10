# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v1.7.0",
        sha256 = "f91a4a87f780b55f8c490dac177fb8474f6f18dd76ed488e78b3795f1d1c1bc4",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":connection.patch",
            ":no_torch.patch",
        ],
        mirrors = mirrors,
    )
