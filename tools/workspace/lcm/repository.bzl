load("//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        upgrade_advice = """
        When updating, lcm needs its own pull request separate from the rest of
        the monthly upgrades.
        """,
        commit = "v1.5.1",
        sha256 = "40ba0b7fb7c9ad06d05e06b4787d743cf11be30eb4f1a03abf4a92641c5b1203",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
