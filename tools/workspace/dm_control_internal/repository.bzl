load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.26",
        sha256 = "1d95095ba8780f582805e9f144d15c67fe618c59bd87e689e66449b440fd13e3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
