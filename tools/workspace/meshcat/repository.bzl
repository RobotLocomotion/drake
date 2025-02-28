load("//tools/workspace:github.bzl", "github_archive")

def meshcat_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "siddancha/meshcat",
        upgrade_advice = """
        Updating this commit requires local testing; see
        drake/tools/workspace/meshcat/README.md for details.
        """,
        commit = "d48277939b7654fd1f1cf62f6630d8f79e5f0faa",
        sha256 = "adeeeb4f41be94862a2946014ee0d475489689fa03d491460b07aa43916b2722",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
