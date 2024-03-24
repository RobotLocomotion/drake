load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "9b274a2570ddefc4140e4b98bcf248003289b870",
        sha256 = "214bf8a757cacea053ed04e666681e34df2e550c5b95df3a6ca8f151a6b550bd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
