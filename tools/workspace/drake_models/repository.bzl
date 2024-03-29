load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "9c377659c3cc3282aa2f2c510c327f4c5538e8fa",
        sha256 = "f6e6ba6e023ec23b2591ff32f026fe43b279c3c86f331a9e6b3bac2fcbe89606",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
