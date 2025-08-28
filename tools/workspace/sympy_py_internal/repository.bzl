load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        commit = "1.14.0",
        sha256 = "813eecbf60fdf4c692cc1cdb30b940072160f4ab0421fa5d7aaa7a8a8c596615",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
