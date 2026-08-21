load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "a03e87bf13502b0b48ebbf2808928fd96ebf9cf3",
        sha256 = "0afd0428f1cefd1ca69a96cc694496abc4cee9f563243c32e038be1abb9f28d1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
