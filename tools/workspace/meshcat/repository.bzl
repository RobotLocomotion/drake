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
        commit = "e87530b45282872a07dd164351aa4dfef40d9f5b",
        sha256 = "d15ba8fa1dca154266bd44265e38bce9fd34db4e5ac9a1dcced639d1c78eef9f",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
