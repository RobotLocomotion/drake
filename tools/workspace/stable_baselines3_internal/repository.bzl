load("//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v2.6.0",
        sha256 = "32813deeea6341be22fa276862f6e9c03b35753a4c8df0b95bffdbb76382ab61",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_torch.patch",
        ],
        patch_cmds = [
            "sed -i -e 's#$#drake_internal#' stable_baselines3/version.txt",
        ],
        mirrors = mirrors,
    )
