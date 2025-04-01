load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "bf04290ac7911fa7a39339c7e507792fd464c438",
        sha256 = "4461d93363716894f5800ab773d855133537fb4580caf87becb78042e2547be1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
