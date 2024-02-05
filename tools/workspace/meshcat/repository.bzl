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
        commit = "92929106db8a75ec85b2689f83e65873ba514fbc",
        sha256 = "5216770e4ddc40e8363e9da9a17afce9271be5f4822015df84cfbe3af477ebbb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
