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
        commit = "6c5687c9da109ba6eae024bc2934013240b8c1db",
        sha256 = "2911883c918b8fd05a83e2cb36bfc560b93b55218b33148ca5995021df71cae7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
