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
        commit = "c0cff8e6990db8e9fb968e6309f9ca16c9dbd3b1",
        sha256 = "98b29421a912230ad75d51a039eb33296216d3d0e1c888a5d413c06646fe6c27",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
