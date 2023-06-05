load("@drake//tools/workspace:github.bzl", "github_archive")

def osqp_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "osqp/osqp",
        upgrade_advice = """
        When updating this commit, see drake/tools/workspace/qdldl/README.md.
        """,
        commit = "v0.6.3",
        sha256 = "a6b4148019001f87489c27232e2bdbac37c94f38fa37c1b4ee11eaa5654756d2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
