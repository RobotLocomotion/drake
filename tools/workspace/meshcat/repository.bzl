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
        commit = "a936c2605f454b99cc02dc53e2b8e70b81ad8373",
        sha256 = "758dabe9953162504a1f43c6d06164d09d8ab183334d755a968949bca17e971c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
