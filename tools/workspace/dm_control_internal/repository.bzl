load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        upgrade_type = "release",
        commit = "1.0.47",
        sha256 = "87e189ebcba1bd9a30e5cb95ca8b149b3175ab7dc317b9e1e90eb4bc099e254f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
