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
        commit = "3228f9d5f775eecf01c5bdfc757cd4a1252c2c0a",
        sha256 = "8f641670c2298b1af3d2493626aac776df90b70f2efa68b8cbf495e5bb988eb1",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
