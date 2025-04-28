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
        commit = "789badf36f25c48c40ff6ee213b12ba4dd520d5a",
        sha256 = "fae1e31f2361da438af1121000de331e27dd7e9884a809643e782fbbc074d732",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
