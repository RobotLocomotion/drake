load("//tools/workspace:github.bzl", "github_archive")

def stable_baselines3_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "DLR-RM/stable-baselines3",
        commit = "v2.3.2",
        sha256 = "99490d5a546861b313bf2d6ae8e20842a2b5512687e5e78a9a2b10c4450c1149",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/no_torch.patch",
        ],
        patch_cmds = [
            "sed -i -e 's#$#drake_internal#' stable_baselines3/version.txt",
        ],
        mirrors = mirrors,
    )
