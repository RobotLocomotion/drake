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
        commit = "0f4af7be1a4311c6835c7106e7935b15174d4236",
        sha256 = "95163929c02b36e2ccc5abedb7febd862fce23bedb479e425c85349048a4ad97",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
