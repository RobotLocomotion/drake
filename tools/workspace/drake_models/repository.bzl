load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "f418885f38be972b560f4f2661792f2800491bd7",
        sha256 = "bacc01fba8f324b8dce1d15ae98083fa7e61463d0d278df46f198c2e3ae14abd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
