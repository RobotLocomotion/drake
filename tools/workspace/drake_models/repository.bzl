load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "c820373584f5f6c3321957f4e22b5cb2fe628713",
        sha256 = "8cace551aec47b4ad64f14a52c09d07d8acaf471e95a1747709d0db9b3a7e446",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
