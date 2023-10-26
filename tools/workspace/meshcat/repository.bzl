load("//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "SeanCurtis-TRI/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "b48a04c7dba9e041e2f46476bd426981b59467b2",
        sha256 = "6287c1deb1eae8a9802984e3d1da0a5ce20008fd9c374322e29e9af201c65c38",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
