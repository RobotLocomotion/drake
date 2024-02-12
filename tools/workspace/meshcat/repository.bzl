load("//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "meshcat-dev/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "d9a2319891af5fbd04525bb4fddce355a5bad48a",
        sha256 = "0e731ef5250ca021a55f33d5e85ca3382d62045a5e3688726b5aa806788ba207",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
