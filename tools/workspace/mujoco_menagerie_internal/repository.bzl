load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "feadf76d42f8a2162426f7d226a3b539556b3bf5",
        sha256 = "3a7a116f4eecf511966c173f5a14bf394c77712f0a5538a2c46f5b1836906c78",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
