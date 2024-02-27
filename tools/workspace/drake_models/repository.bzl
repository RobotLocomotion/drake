load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "accdf59bc4227b4be8a9867b63058083003d1168",
        sha256 = "b0dc30fc1d3410afdd66d5d7b5370c5df1a814d66197f0934103c7e89b191739",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
