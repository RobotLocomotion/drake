load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        commit = "sympy-1.12.1",
        sha256 = "66d863c8be3499d0957f063a4bea2b844f4135840b6a29d802e2297631e50e99",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
