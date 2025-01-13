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
        commit = "v1.5.0",
        sha256 = "590a7d996daa3d33a7f3094e4054c35799a3d7a4780d732be78971323e730eeb",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
