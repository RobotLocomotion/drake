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
        commit = "e5be32269361fe64d03b77ba0b6ac8f2ff8fee72",
        sha256 = "072c5751a5a06f0c7547a465c5dd0a90cdcacb6a9638e5d214640cf3fffe51b4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
