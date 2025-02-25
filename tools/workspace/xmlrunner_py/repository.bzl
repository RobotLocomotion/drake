load("//tools/workspace:github.bzl", "github_archive")
load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

def xmlrunner_py_repository(
        name,
        mirrors = None,
        _is_drake_self_call = False):
    if not _is_drake_self_call:
        print_warning("xmlrunner_py_repository")
    github_archive(
        name = name,
        repository = "xmlrunner/unittest-xml-reporting",
        commit = "3.2.0",
        sha256 = "be4607e18c4f3bb103f743fc1d98acec095d2cc570d69a6fd5db1c1a4decc86c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
