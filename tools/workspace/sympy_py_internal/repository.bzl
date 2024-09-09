load("@drake//tools/workspace:github.bzl", "github_archive")

def sympy_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "sympy/sympy",
        commit = "1.13.2",
        sha256 = "e84fb33d724c8b27e5a543bf352efca310ac25a9fb39012ce2edbc5e14c6719c",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
