load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")

# This must be a module extension (instead of a respository rule) so that the
# new_local_repository path resolves correctly w.r.t Drake's root.

def spgrid_module_extension_impl(module_ctx):
    license = Label("@drake//third_party:spgrid/LICENSE")
    directory = module_ctx.path(license).dirname
    new_local_repository(
        name = "spgrid_internal",
        path = str(directory),
        build_file = "//tools/workspace/spgrid_internal:package.BUILD.bazel",
    )
