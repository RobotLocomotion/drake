load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        upgrade_type = "release",
        commit = "1.0.43",
        sha256 = "7698a9509b078cc9c9fce37feca69daec94cf792f015029511378b06026dc4c8",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
