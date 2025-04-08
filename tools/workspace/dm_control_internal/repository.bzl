load("//tools/workspace:github.bzl", "github_archive")

def dm_control_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "deepmind/dm_control",
        commit = "1.0.28",
        sha256 = "78720a9442d9f7adebf73f362a42526b5c4fc5aeba75484380672ef1d38fa028",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
