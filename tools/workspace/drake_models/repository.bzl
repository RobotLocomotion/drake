load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "199f3e5a8c868ed459f05e73e5eee65da843ffdd",
        sha256 = "c09e320a42d7ce6db2e1f622898726c5853db5ee0faf23715f9d3c2274a53ac7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
