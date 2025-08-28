load("//tools/workspace:github.bzl", "github_archive")

def gklib_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "KarypisLab/GKlib",
        commit = "6e7951358fd896e2abed7887196b6871aac9f2f8",
        sha256 = "42c52e58408ad06dc23c2a052eea529b269287ea8a6f3f6cdc47f18d50b2a177",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
