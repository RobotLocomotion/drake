load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "5e42b0eb3c11be56e31c8d94637ea7f8abca0f96",
        sha256 = "6a1d370b3f2d093b5b7f5dd7defdc955d6f76c07d2efb295b476282cca141db1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
