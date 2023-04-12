load("@drake//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v1.8.0",
        sha256 = "2ac876fc53546258008dbb1d249eb5b051bf9f0c8d1aae88c0c75af08c1c180d",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":connection.patch",
            ":no_torch.patch",
        ],
        mirrors = mirrors,
    )
