load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "c35048b8df1554495da561dc9c172caa6d79da58",
        sha256 = "e91dc5196beb70e48b9ec1a2da5a23d314e40b37378eb75298a2f13f23d80ed5",
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
