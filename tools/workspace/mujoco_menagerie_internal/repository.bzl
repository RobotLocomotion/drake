load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "469893211c41d5da9c314f5ab58059fa17c8e360",
        sha256 = "1cfe0ebde2c6dd80405977e0b3a6f72e1b062d8a79f9f0437ebebe463c9c85f7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
