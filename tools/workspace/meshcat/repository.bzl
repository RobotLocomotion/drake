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
        upgrade_type = "commit",
        commit = "ad72d372dcdd52cdf3a177bed49cb20cb2951217",
        sha256 = "1454ff074f53f11c4b7248bb14218b81f848c7fcfb075e37938ff77146800838",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
