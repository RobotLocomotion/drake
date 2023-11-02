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
        commit = "fa29aecebd5f1712844c3b98967a7d21707df5a0",
        sha256 = "bdd505e90ab4e46958714a909f0a45d42d905eee0fa87637b97092035a4df7a3",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
