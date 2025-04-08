load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "8268a177a969809dda4e086d4888acd1d59bdf24",
        sha256 = "ca3385bda7cb661db7efcc77c249821fdef5eabf4146650a76f6f3ee754903c8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
