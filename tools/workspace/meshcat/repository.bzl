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
        commit = "4b4f8ffbaa5f609352ea6227bd5ae8207b579c70",
        sha256 = "2f7131aa47c38d21f7904ecc07344bb12538d495668682117e0e7fbd28ad893b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
