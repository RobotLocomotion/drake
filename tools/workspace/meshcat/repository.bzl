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
        commit = "bbdc1d95059b3596c2bb12e34796b17731acb912",
        sha256 = "ab65ee5d361a2c0ced0afea55e0aa731421412f37c089c97029eaafc1d9ee884",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
