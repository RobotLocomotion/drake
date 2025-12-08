load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "4b5f248fc29462a5c121b079244c63faa9fa2622",
        sha256 = "eb151bae64e9ee188cc49c911242d3943a6d9bc714bb3c5f3df0ad8259dbd887",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
