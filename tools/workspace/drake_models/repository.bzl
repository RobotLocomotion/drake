load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "fb74187a9c5e1c2bdbdda3db90adc986684040f5",
        sha256 = "17b307a246e5addc57500172c67c1a69a873bc4ffedcf3c7563404a6eccaceea",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
