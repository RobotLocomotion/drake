load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "4693b86786a9708d6c6ba148a388ab9dca17be8d",
        sha256 = "941026fcf09783a46b01e7a39f3fa258e47fd34701dfb31347e0a69fd4c269e7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
