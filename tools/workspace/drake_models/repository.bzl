load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "78cad6a331aa0610e8f84e463d28103d096bdb83",
        sha256 = "a7e69562fa97fcbfa19562880d7ad4764e4cf7d4da9db359ecd5a62dc1963249",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
