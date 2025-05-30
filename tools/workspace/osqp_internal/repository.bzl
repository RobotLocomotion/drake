load("//tools/workspace:github.bzl", "github_archive")

def osqp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osqp/osqp",
        upgrade_advice = """
        When updating this commit, see
        drake/tools/workspace/qdldl_internal/README.md.
        """,
        commit = "v1.0.0",
        sha256 = "dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
