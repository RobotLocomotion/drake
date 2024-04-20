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
        commit = "3ce3dd6e60d68ed77619158a5d9b2c991022dbd4",
        sha256 = "27cebc2bd6b19cd0126bcb288403483f73c693c928c57a439918953b29bf4130",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
