load("//tools/workspace:github.bzl", "github_archive")

def qdldl_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osqp/qdldl",
        upgrade_advice = """
        When updating this commit, see
        drake/tools/workspace/qdldl_internal/README.md.
        """,
        commit = "v0.1.9",
        sha256 = "7d1285b2db15cf2730dc83b3d16ed28412f558591108cca4f28d4438bf72ceed",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
