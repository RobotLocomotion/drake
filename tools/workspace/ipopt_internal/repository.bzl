load("//tools/workspace:github.bzl", "github_archive")

def ipopt_internal_repository(
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
        commit = "releases/3.14.17",
        sha256 = "17ab8e9a6059ab11172c184e5947e7a7dda9fed0764764779c27e5b8e46f3d75",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
