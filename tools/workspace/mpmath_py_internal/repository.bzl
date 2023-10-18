load("@drake//tools/workspace:github.bzl", "github_archive")

def mpmath_py_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "mpmath/mpmath",
        commit = "1.3.0",
        sha256 = "8f702663fa422fbbf02d15792da4b2566160df35b8a4af55fe64cba2fec2aa00",  # noqa
        build_file = ":package.BUILD.bazel",
        mirrors = mirrors,
    )
