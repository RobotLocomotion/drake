load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "db9c78f538fb1cd519098a6d6422c3f179415eb6",
        sha256 = "f3a630b78671f52412e2b5b54dc4ac26c855a88af8c49df0e2f9a9d36f20b570",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
