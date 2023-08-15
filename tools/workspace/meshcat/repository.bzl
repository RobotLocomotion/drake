load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "db6323bef9a2bad56046fa5ed30bfab36f1551d5",
        sha256 = "c1a9df479d74a82e980adc924a8e63d99011f728becd9d2557d99257ab16b44a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
