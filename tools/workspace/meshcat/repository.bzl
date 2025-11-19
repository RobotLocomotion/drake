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
        commit = "f7b8588f73aa14bffe52f662fb0ba04f4f1d4f22",
        sha256 = "79dfc209d1c7330fcf6bf323074a8d01cb5627c97d989ccb34b7e8aba165f36e",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
