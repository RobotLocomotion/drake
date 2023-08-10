load("@drake//tools/workspace:github.bzl", "github_archive")

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
        commit = "d9245235a0dd819a6d4c07a0e970ff7403edde67",
        sha256 = "34e5892257e2c94ee1782c7cc7a6476303a490ec093e4ed76e900a5ffae06f15",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
