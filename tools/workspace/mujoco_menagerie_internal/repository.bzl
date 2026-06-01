load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        upgrade_type = "commit",
        commit = "4c358ef9d9d7f32ca58b40b490884a0c1726a440",
        sha256 = "d57b869703b30890c6b322793ca4ac8b7e4d1e841a1155912262823d3db04f6c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
