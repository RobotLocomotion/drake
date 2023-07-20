load("@drake//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "rdeits/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "120cbc1f57b10a15efac2b79f38f19b8bdd44722",
        sha256 = "f8a63d3453470d19fd2667c551ae2bde99c9afacd3091c73f960eeff211273fa",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
