load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.18",
        sha256 = "79d7c6b2b0417ed8f67ae6106fc2933069da45caa6af71676778b7cc99f32131",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
