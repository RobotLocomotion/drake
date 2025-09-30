load(
    "//tools/workspace/crate_universe/lock/details:crates.bzl",
    "crate_repositories",
)

def _crate_licenses_repository_impl(repo_ctx):
    my_dir = "@drake//tools/workspace/crate_universe"
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
    root_module_direct_deps = ["crate_licenses"]
    direct_deps = crate_repositories()
    root_module_direct_deps.extend([repo.repo for repo in direct_deps])
    return module_ctx.extension_metadata(
        root_module_direct_deps = root_module_direct_deps,
        root_module_direct_dev_deps = [],
    )

crate_universe = module_extension(
    doc = "Internal use only.",
    implementation = _impl,
)
