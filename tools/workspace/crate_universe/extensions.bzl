load(
    "//tools/workspace/crate_universe/lock/details:crates.bzl",
    "crate_repositories",
)

def _license_repository_impl(repo_ctx):
    my_dir = "@drake//tools/workspace/crate_universe"
    repo_ctx.symlink(
        Label(my_dir + ":BUILD.crate_licenses.bazel"),
        "BUILD.bazel",
    )
    repo_ctx.symlink(
        Label(my_dir + ":lock/repo_names.bzl"),
        "lock/repo_names.bzl",
    )

license_repository = repository_rule(
    implementation = _license_repository_impl,
)

def _impl(module_ctx):
    direct_deps = crate_repositories()
    license_repository(name = "crate_licenses")
    return module_ctx.extension_metadata(
        root_module_direct_deps = [repo.repo for repo in direct_deps] + [
            "crate_licenses",
        ],
        root_module_direct_dev_deps = [],
    )

crate_universe = module_extension(
    doc = "Internal use only.",
    implementation = _impl,
)
