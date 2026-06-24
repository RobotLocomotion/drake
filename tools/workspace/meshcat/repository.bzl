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
        upgrade_type = "commit",
        commit = "57f6781a2d8f8737be36716e6aaa061ee159a7c4",
        sha256 = "21b7d5d37250d500e2e9983a10ef8a7bbe0351c5766e7066780f79d949cc1d14",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
