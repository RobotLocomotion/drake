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
        commit = "v0.1.8",
        sha256 = "ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
