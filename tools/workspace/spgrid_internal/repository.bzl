load("@bazel_tools//tools/build_defs/repo:local.bzl", "new_local_repository")

def _module_extension_impl(module_ctx):
    license = Label("@drake//third_party:spgrid/LICENSE")
    directory = module_ctx.path(license).dirname
    new_local_repository(
        name = "spgrid_internal",
        path = str(directory),
        build_file = "//tools/workspace/spgrid_internal:package.BUILD.bazel",
    )

spgrid_internal_module_extension = module_extension(
    implementation = _module_extension_impl,
)
