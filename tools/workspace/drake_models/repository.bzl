load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "512978dc9819b8f595bbbd701b90cba2c34caf53",
        sha256 = "b43620c2579d63929c41ab8c0120c41d5877fb64ddbcc967b99f208c838e7476",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
