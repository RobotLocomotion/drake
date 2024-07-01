load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "af493511dbdfce2e046858a4d2f2955e063e17fd",
        sha256 = "2975d9a03728bf1b8a2b7d2b14e62bc8d411031b73264e797dcce649bbf4811c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
