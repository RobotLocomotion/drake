load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.20",
        sha256 = "c9aba53ed7c5687966b986509ecee7f5162d9615585056a1e23b361f05fcfb20",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
