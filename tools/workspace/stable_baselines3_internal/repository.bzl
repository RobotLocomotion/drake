# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v1.6.2",
        sha256 = "b3d0ae707f9e1ae6429a705dddaa79bc439e8647149b4b5f353ee93db3f08fa7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":connection.patch",
            ":no_torch.patch",
        ],
        mirrors = mirrors,
    )
