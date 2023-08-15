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
        commit = "87305ff7513462be06457b34739dcd99ae983b6b",
        sha256 = "00356abf0aeea53b34fa0651785fd6374f60dddf133c2f078eac78584ed1435b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
