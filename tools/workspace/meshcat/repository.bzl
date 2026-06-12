load("//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        local_repository_override = "/home/seancurtis/code/meshcat",
        name = name,
        repository = "SeanCurtis-TRI/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "815addc85f3bca5615ece3e8095bbca009480143",
        sha256 = "0a527248e95f31f8723817e1b678a97827c5293d3fd30b600e383ea09bf6063f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
