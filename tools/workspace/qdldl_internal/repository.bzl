load("@drake//tools/workspace:github.bzl", "github_archive")

def qdldl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osqp/qdldl",
        upgrade_advice = """
        When updating this commit, see drake/tools/workspace/qdldl/README.md.
        """,
        commit = "v0.1.6",
        sha256 = "aeb1b2d76849c13e9803760a4c2e26194bf80dcc9614ae25ca6bcc404dc70d65",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
