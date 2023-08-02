load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "d6f9cb4e4a616d1e1d3bd8944bc89541434f1d49",
        sha256 = "bd17f71f1f910196332b969ec87c02b5fdbc309da0c1d488bc569b5fb3bddc4b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
