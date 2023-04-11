load("@drake//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "5dec8898a4539589d4989f6553673e588e6a6b06",
        sha256 = "9866c4d3da7098a1b36d7b37d0f7c65f00e47bb979d3146cbc39aef59ae09b5e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
