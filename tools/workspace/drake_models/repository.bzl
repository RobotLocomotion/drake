load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "54684a01dad22d42a24401c7666dcd3bb1739499",
        sha256 = "ab9c8efcff1dffa5347da24d1079a39a1c59510bd8e8f909fe1c73dfe1c0f9a1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
