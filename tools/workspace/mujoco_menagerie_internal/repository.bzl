load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "e7cb46f0715e72c6e11e1a6f96ccd4b4022797a2",
        sha256 = "d23ea996b68bacf8271c7e077b25407c1d37440ea48ab65e6ea0c52f3699cf1e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
