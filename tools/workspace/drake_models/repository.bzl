load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "d2d316acbf12b056a3c8ba372d1c665a053fc1ac",
        sha256 = "3c5a33b6b3cb9e7d52b706e42a940dd807527bedcfb55cf8ca2872dda0c1a0f7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
