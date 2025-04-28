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
        commit = "109a4e21c348cd7979a5ca243f8b9a34b4c06273",
        sha256 = "eb38095f527d99987accacc433d7350126547301851576712c69bc4741afe460",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
