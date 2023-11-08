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
        commit = "533779e3bc392ac51b4a3a9a3b74ac81df3ae2f0",
        sha256 = "879e9ab7913270e87190342813e2c5181e690459026662200a6b19fedf3265ea",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
