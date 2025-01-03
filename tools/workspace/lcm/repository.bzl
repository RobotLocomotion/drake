load("//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        upgrade_advice = """
        When updating, the version numbers within the two lcm-*.cmake files in
        this directory must also be updated to match the new version.
        """,
        commit = "v1.5.1",
        sha256 = "40ba0b7fb7c9ad06d05e06b4787d743cf11be30eb4f1a03abf4a92641c5b1203",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
