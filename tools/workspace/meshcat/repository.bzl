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
        commit = "df31f9c357ba37121dc38d463ac94780204a3e37",
        sha256 = "b0d8210e11495c1c6011927a9b0a76b1940e8d97e44044124333f1d19cb4aecd",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
