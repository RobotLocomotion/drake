load("//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v2.8.0",
        sha256 = "9d8cbcf140d725d337ce1af06cd497ddfe2ae91fe175e9d97c2a08edfb4f7a2a",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_torch.patch",
        ],
        patch_cmds = [
            "sed -i -e 's#$#drake_internal#' stable_baselines3/version.txt",
        ],
        mirrors = mirrors,
    )
