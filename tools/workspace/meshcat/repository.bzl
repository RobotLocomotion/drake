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
        commit = "10a4338776fc8792df29c25a5e8a6b42bfd6879a",
        sha256 = "635a79485826f496a351aad0f266b212753ffed33a5c9d1107540ad9d033d959",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
