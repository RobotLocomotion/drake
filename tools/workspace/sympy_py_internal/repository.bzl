load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        commit = "1.13.1",
        sha256 = "b5420b9ba015803ae50048602e5e97af1e428649e10dc872fc3e344a98fd9f34",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
