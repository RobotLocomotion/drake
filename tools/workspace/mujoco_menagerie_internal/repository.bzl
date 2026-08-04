load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        upgrade_type = "commit",
        commit = "accb6df40a9a1d1e49eff88157f6818b63a49335",
        sha256 = "766c45e0a88f49fa9298fdc27d36c7c5a51eede30f0cdde0ed801593c51b5f28",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
