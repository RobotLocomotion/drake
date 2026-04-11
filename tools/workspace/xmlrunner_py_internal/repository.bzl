load("//tools/workspace:github.bzl", "github_archive")

def xmlrunner_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "xmlrunner/unittest-xml-reporting",
        commit = "4.0.0",
        sha256 = "5fd1880a41247c4976a73699598225b80e597755791ba76f2c5066142e1cd923",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
