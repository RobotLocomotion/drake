# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def lcm_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "lcm-proj/lcm",
        upgrade_advice = """
        When upgrading this commit, check if the LCM maintainers have tagged
        a new version number; if so, then update the version numbers within
        the two lcm-*.cmake files in this directory to match.
        """,
        commit = "0289aa9efdf043dd69d65b7d01273e8108dd79f7",
        sha256 = "45a2b8376ee1de4e3150b2b1afc7ad721bff5bac4ddfe19e1916b50060fd8ee5",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
