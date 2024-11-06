load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "9e9185d772df5812346ae4668def8117b354bbae",
        sha256 = "5c7e9625041c7df28739c9ff12d937272a738eba15ca5838654ce71ad68a13c4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
