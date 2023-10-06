load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        commit = "sympy-1.12",
        sha256 = "512d5084f900e6f340b4d7b7d95b50127e2d023746584f81ce8c67e09341ee5a",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
