load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "378851581f6729c0937a6c37ceefafd6a14c4e4d",
        sha256 = "5d4f749e72cd83605ccb48bfcbfeb3e688d03a284eb971ab72635d0b45697380",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
