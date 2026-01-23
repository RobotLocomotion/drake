load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "39616e33d0b0953a540f655bed6791e75f233502",
        sha256 = "db695a13fc7c493b40d49e7016be0b603aaf951fdb4a3325194c1032bd4629a7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
