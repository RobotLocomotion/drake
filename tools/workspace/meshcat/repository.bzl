load("@drake//tools/workspace:github.bzl", "github_archive")

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
        commit = "7b233bc98717d82212312af37ff7d0ce9f51a1ba",
        sha256 = "49b1a985d847e67de0efdb32f268e704c4fc1c2c1fc4769715cd3d87601d1aa6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
