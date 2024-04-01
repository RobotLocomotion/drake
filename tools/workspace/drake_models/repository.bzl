load("//tools/workspace:github.bzl", "github_archive")

def drake_models_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "RobotLocomotion/models",
        commit = "00fcfc2053ca957e42140c5533bb66e34c0c7e41",
        sha256 = "adda88c50162d72d4d3d311f1ca682e6713e99ca01a09ace2d5f756f041948fe",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
