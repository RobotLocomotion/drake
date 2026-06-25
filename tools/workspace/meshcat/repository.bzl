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
        upgrade_type = "commit",
        commit = "f9dd4ccd798fe1791a8e44e417880ae31312d9a2",
        sha256 = "d372c9ded530887725564bed47a5933cf195e990c8897a1fa131a013994680bc",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
