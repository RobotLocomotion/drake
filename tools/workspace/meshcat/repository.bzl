load("@drake//tools/workspace:github.bzl", "github_archive")

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
        commit = "44eac463725f048c47debfe34d3f935d01aa6bac",
        sha256 = "7ac5e9fdcc407abb4770bad1cee849de939f1dc856ef38d601a1982abec68ac3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
