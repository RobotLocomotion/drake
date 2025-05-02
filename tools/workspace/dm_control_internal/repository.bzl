load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.30",
        sha256 = "1176ab4f5b281c0a9577125288882399bab77f06f0f4538f8b6e67f0d74ee494",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
