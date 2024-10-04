load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        commit = "1.13.3",
        sha256 = "3dbbbd837ce027285081d1497e8c6aab3bbd197dc7d8990c3f367de99436a6c2",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
