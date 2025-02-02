load("//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v2.5.0",
        sha256 = "1906a018ee76b71ccc5c774d6a3fcc79d34b5a51b33ba7edfccca40a7f5bc5f7",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_torch.patch",
        ],
        patch_cmds = [
            "sed -i -e 's#$#drake_internal#' stable_baselines3/version.txt",
        ],
        mirrors = mirrors,
    )
