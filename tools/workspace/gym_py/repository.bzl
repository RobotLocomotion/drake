load("@drake//tools/workspace:github.bzl", "github_archive")

def gym_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "openai/gym",
        commit = "v0.21.0",
        sha256 = "0efc4ca01fa0d0cd10391b37db0c52e09084fb7a3fd04cdc68e08081acbf4418",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
