load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "66384c6b8581c811a7b1eb63bcf4fa944fa43602",
        sha256 = "1eaa71d81a2179f774eb56689045cc9c7d1779df64fef60e120d55928df07ff7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
