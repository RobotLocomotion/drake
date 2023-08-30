load("@drake//tools/workspace:github.bzl", "github_archive")

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
        commit = "0e30a85f5a9763ecd75a6c87bc094b45b159d8c4",
        sha256 = "b7148e88a555699b51030db65a22566b74e460a476c39240f6ee04391d213be4",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
