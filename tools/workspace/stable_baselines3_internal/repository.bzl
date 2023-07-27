load("@drake//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v2.0.0",
        sha256 = "cab35dbad87792d8a64dce015b9817c3d2de2bdeb6d57c95d459f27d65d7c21e",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_torch.patch",
            ":patches/version.patch",
        ],
        mirrors = mirrors,
    )
