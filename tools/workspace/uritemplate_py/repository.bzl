# -*- python -*-

load("@drake//tools/workspace:github.bzl", "github_archive")

def uritemplate_py_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "python-hyper/uritemplate",
        commit = "3.0.0",
        sha256 = "733fef5b17c4d9bb86b52fcd5c97a008bfc2fcfdbc1d04a06b2e881935944b0e",  # noqa
        build_file = "@drake//tools/workspace/uritemplate_py:package.BUILD.bazel",  # noqa
        mirrors = mirrors,
    )
