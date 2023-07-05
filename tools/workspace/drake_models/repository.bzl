load("@drake//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RussTedrake/models",
        commit = "4b16cd5227dd1d568de467703b6c659e38886ee3",
        sha256 = "c2af53e43825e5a2ea19368e380f6a1295cb9db3e2c093bf095e77d07e09838f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
