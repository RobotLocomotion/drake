load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.15",
        sha256 = "bac091b18689330a99b7c18ddf86baa916527f5e4ab8e3ded0c8caff1dab2048",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
