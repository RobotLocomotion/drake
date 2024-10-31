def spgrid_internal_repository(name):
    native.new_local_repository(
        name = name,
        path = "third_party/spgrid/",
        build_file = "//tools/workspace/spgrid_internal:package.BUILD.bazel",
    )
