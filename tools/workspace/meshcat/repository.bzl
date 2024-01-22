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
        commit = "0969de3bff97c9943b5ca53836e18302af905216",
        sha256 = "18f0f8b9b8c0d75237a926173592455923d09068c9dea96443a99088d50129a7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
