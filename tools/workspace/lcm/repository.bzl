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
        commit = "ab70eaff2bf9e7da097ae3f6183da7c234a286f1",
        sha256 = "21a93b77bc20025070c6cb9c2d1da30e85ede5d503570130822743768ee2c2ed",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
