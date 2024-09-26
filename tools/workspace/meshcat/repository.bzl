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
        commit = "70bcdc1134bde82d183f967e7b97dd3b5c4d67eb",
        sha256 = "e04e3a8f341b3963c8f2bcc5091132249afc37467058d70e750e970303a4d8ec",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
