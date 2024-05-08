load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "26a9827135e53c337daa8cb30c4cf1f14045b93b",
        sha256 = "24a3841363ddfa159e9ab99ccc2db2f3c175d522a3cc0df308040849f333ac70",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
