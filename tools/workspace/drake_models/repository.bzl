load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "2a6555d644e7d67468a9f32c51c78dcf204f37d2",
        sha256 = "331d44d4b6afefb535e9410c2b79f8c2fd327a59155dc451aef101e88c27fd92",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
