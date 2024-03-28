load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "8f4cb421e0cb300e12b6ac8fb7b1cbd1967fc24a",
        sha256 = "ad1f685f584831a9a176307acb86d3233f124988971e40b241573b339664ec2d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
