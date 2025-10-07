load("//tools/workspace:github.bzl", "github_archive")

def xmlrunner_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "xmlrunner/unittest-xml-reporting",
        commit = "3.2.0",
        sha256 = "be4607e18c4f3bb103f743fc1d98acec095d2cc570d69a6fd5db1c1a4decc86c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
