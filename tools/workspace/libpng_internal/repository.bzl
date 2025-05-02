load("//tools/workspace:github.bzl", "github_archive")

def libpng_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "glennrp/libpng",
        commit = "v1.6.48",
        sha256 = "b17e99026055727e8cba99160c3a9a7f9af788e9f786daeadded5a42243f1dd0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
