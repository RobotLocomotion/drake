load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "ef0e2acef896ce38ec3ea179c64dae27460899b3",
        sha256 = "03dbaa291b58aaa7a88379e13143da22e2000d3c8d3737ef37899d3b8a0be0eb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
