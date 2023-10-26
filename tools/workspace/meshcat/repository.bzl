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
        commit = "73f2fb15a1542a6a79aeb5efb10ced23afc116e7",
        sha256 = "cdbd90af34d7e9be4406f4e4b4b1677b7fcd65f3133d9ad054935019d5bc6a77",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
