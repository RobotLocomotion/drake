load("//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "SeanCurtis-TRI/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "baf6298c8dd11c37d7ccb67c296219b4fc8d3502",
        sha256 = "05e0f5faecc3397b51b184c946e74b05a1b0ffc1845b143ed2da38c3e7f5fae3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
