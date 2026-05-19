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
        commit = "5724ffd12c8584f5e8548ca4faaa49c098377ae0",
        sha256 = "c7015e3449ff3753dff88194d65524e982b79042210a2d6373223369d635e61b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
