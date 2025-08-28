load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "7b92aacbe021861ec9bbbb82d8ab9a19ded970ff",
        sha256 = "719c542a6323d5a8cc74fe6da84f305aebc6d7810b84cb94023d80f78e06586b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
