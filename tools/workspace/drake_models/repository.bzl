load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "deee02d4d79937e112cb428e638e7ee44d51305f",
        sha256 = "9f5bc1b923e4de584a030d17c1fcfedfc1ca29e805c453156cab297e6bb89054",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
