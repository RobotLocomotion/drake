load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "ccbaf9193ea147ad963558549202299ac07c69dc",
        sha256 = "fb4199fab0c91be35f802d9c0ff4d59e3862d827c53d2928266f20485aaa6cd2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
