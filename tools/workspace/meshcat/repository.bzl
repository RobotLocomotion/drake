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
        commit = "c40e062d1b9156562f57666ca30d1d55482702cf",
        sha256 = "a28b284202a95823ddd5cf2ba59944475ecf0d2fa83f54cc931a05a3600601bf",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
