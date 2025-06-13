load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "cfd91c5605e90f0b77860ae2278ff107366acc87",
        sha256 = "03ecaff3f29a1ce2f5c1f164ec9b0c9b6c768307d4ad5499a0629ff0b6b5ea89",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
