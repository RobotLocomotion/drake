load("//tools/workspace:github.bzl", "github_archive")

def mujoco_menagerie_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "google-deepmind/mujoco_menagerie",
        commit = "39664247032283fc5cda18cf6ce2b28cdc84eb61",
        sha256 = "44b6568070ef5318dad8ecca4f8c20803ed42221f55c6047f37add8a7581486d",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
