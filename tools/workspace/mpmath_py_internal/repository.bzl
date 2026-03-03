load("@drake//tools/workspace:github.bzl", "github_archive")

def mpmath_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mpmath/mpmath",
        commit = "1.4.0",
        sha256 = "ac7c7cbda2a489e3b564934f6a14ddce30c4a65904c3da9e3cef3c549b3b55c0",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
