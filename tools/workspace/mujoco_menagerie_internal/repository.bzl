load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "bf756430b615819654b640f321c71ba5c3ebeef8",
        sha256 = "cdce25baeb94c8fc1d7cf326530695a967cb77dbca7c0e64c2371ec555b04ac2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
