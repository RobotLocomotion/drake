def _crate_licenses_repository_impl(repo_ctx):
    my_dir = "@drake//tools/workspace/crate_universe"
    manifest = repo_ctx.path(
        Label("@clarabel_cpp_internal//:rust_wrapper/Cargo.toml"),
    )
    repo_ctx.symlink(manifest, "Cargo.toml")
    repo_ctx.symlink(manifest.dirname.get_child("src"), "src")
    repo_ctx.symlink(Label(my_dir + ":lock/Cargo.lock"), "Cargo.lock")
    repo_ctx.symlink(
        Label(my_dir + ":BUILD.crate_licenses.bazel"),
        "BUILD.bazel",
    )
    repo_ctx.symlink(
        Label(my_dir + ":lock/repo_names.bzl"),
        "lock/repo_names.bzl",
    )

crate_licenses_repository = repository_rule(
    implementation = _crate_licenses_repository_impl,
)

def _impl(module_ctx):
    crate_licenses_repository(name = "crate_licenses")
    return module_ctx.extension_metadata(
        root_module_direct_deps = ["crate_licenses"],
        root_module_direct_dev_deps = [],
    )

crate_universe = module_extension(
    doc = "Internal use only.",
    implementation = _impl,
)
