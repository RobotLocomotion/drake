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
        commit = "601b55df0839430297ae53503ea2c61e4cdf1314",
        sha256 = "c1b006483773ad435acba17def30d3359eb39bf7c859fcfbcfd8baca006def37",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
