load("@drake//tools/workspace:github.bzl", "github_archive")

def tomli_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "hukkin/tomli",
        commit = "2.0.1",
        sha256 = "ad22dbc128623e0c156ffaff019f29f456eba8a5d5a05164dd34f63e560449df",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
