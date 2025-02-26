load("//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "siddancha/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "14f683e4bcd5bb34b435b4c74309e0e50d54bc9a",
        sha256 = "dae769f555fc7281b8a30795a9a5978faef3ec85ceafb809c332575dbe0b52d7",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
