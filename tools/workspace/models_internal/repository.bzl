load("@drake//tools/workspace:github.bzl", "github_archive")

def models_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "dfd7adf36c3845175745a82786b9cc49970b0a5e",
        sha256 = "5c8a7a7bda819f25381968208552aaa1c427e30a1055f971cfb951f8159c59c3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
