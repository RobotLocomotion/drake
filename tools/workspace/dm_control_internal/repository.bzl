load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.25",
        sha256 = "2d577f89be8d7ff4c012653fff3879fa5fabc350bd77d58885317fafdf759ff6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
