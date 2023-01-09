# -*- python -*-

load(
    "@drake//tools/workspace:github.bzl",
    "github_archive",
)

def msgpack_internal_repository(
        name,
        mirrors = None):
    github_archive(
        name = name,
        repository = "msgpack/msgpack-c",
        commit = "cpp-4.1.3",
        sha256 = "fd0a685656f11b8aa09ed33bcbdcad3105d25d0034ca3dba9fe957623a42d253",  # noqa
        build_file = ":package.BUILD.bazel",
        patches = [
            ":patches/vendor.patch",
        ],
        mirrors = mirrors,
    )
