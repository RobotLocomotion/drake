load("@drake//tools/workspace:github.bzl", "github_archive")

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
        commit = "b5a24ea2219d5ae7fb08652d768564a4338dcdbd",
        sha256 = "79abef63a3b2b1a9afd86ea7c78e253bcca5b8d23c33094075c102d28373439b",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
