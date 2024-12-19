load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")

def spgrid_internal_repository(name):
    new_local_repository(
        name = name,
        path = "third_party/spgrid/",
        build_file = "//tools/workspace/spgrid_internal:package.BUILD.bazel",
    )
