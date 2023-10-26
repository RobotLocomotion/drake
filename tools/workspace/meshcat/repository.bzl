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
        commit = "c788d692d62bf63408be3a127a6288c9845b4325",
        sha256 = "0ac908b62cc77fcb9b0fc9fa55e04eeee3eda3e94045e85ccef556cb0cb6271a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
