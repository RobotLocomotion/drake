# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def xmlrunner_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "xmlrunner/unittest-xml-reporting",
        commit = "3.0.2",
        sha256 = "dbe165386952ec5373d4db5b4ac0644b60b734f4b02b9e575b1d0dc873616ba4",  # noqa
        build_file = "@drake//tools/workspace/xmlrunner_py:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
