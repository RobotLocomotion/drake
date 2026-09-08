load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        upgrade_type = "release",
        commit = "1.0.45",
        sha256 = "ab556f102a54c3b972cc85a77345e8707c93fc751f2ebb85f6aaa4b8a313e7f6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
