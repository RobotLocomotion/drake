load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_fromsource_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "coin-or/Ipopt",
        upgrade_advice = """
        When updating, the ipopt version in
        tools/wheel/image/dependencies/projects.cmake
        must also be updated.
        """,
        commit = "releases/3.14.14",
        sha256 = "264d2d3291cd1cd2d0fa0ad583e0a18199e3b1378c3cb015b6c5600083f1e036",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
