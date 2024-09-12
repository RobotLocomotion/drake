load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "15b7a2b9605fa7ec6de42828aca64f00ffa86acb",
        sha256 = "b1474cc79d44ad4bec010dfcf4c937e58772b5f13f467f8826a525a4852ac1f6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
