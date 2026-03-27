load("//tools/workspace:github.bzl", "github_archive")

def implib_so_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "TaylorSasser/Implib.so",
        commit = "a5116d3d2138a985800e2754ae95244c17f78349",
        sha256 = "55ddbb9e1486ab15de2141b26fe037fb9dc6ea251f90812a569eab6007413bd9",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
