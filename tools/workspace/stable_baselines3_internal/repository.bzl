load("//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        upgrade_type = "release",
        commit = "v2.9.2",
        sha256 = "a0bedf2af496e1bf124585cefa3f9c8f27e3b17d74c5beca64198bcccda617bc",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_torch.patch",
        ],
        patch_cmds = [
            "sed -i -e 's#$#drake_internal#' stable_baselines3/version.txt",
        ],
        mirrors = mirrors,
    )
