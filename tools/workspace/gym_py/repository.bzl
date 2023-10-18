load("//tools/workspace:github.bzl", "github_archive")

def gym_py_repository(
        name,
        mirrors = None):
    """The @gym_py external is deprecated and will be removed from Drake's
    WORKSPACE on or after 2023-12-01; see @gymnasium_py for an available newer
    replacement
    """
    github_archive(
        name = name,
        repository = "openai/gym",
        commit = "v0.21.0",
        sha256 = "0efc4ca01fa0d0cd10391b37db0c52e09084fb7a3fd04cdc68e08081acbf4418",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
