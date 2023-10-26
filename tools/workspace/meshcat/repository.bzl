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
        commit = "8c49b8aaa227469e3ad3d327c2ed54228ef73946",
        sha256 = "18f41e149025444d90e6533ad3134a08ebc223df3c7a8e8656d2605f73cf5eb6",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
