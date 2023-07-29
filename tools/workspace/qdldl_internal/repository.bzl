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
        commit = "v0.1.7",
        sha256 = "631ae65f367859fa1efade1656e4ba22b7da789c06e010cceb8b29656bf65757",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
