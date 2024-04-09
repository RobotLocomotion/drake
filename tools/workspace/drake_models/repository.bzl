load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "4a43724fc3973da65d18263db3b2dfc86403f56c",
        sha256 = "6afd8a3a834100f56c14e8c108b522cb73229fb34da83745e9cab4ac8aafc4f1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
