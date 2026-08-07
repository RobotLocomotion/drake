load("@crates//:defs.bzl", _all_crate_deps = "all_crate_deps")
load("@rules_rust//rust/private:providers.bzl", "CrateInfo")
load("//tools/install:install.bzl", "InstallInfo")

def all_crate_deps():
    """Returns the fully qualified labels of all crate_universe crates."""
    return _all_crate_deps()

def _install_crate_license_impl(ctx):
    package_directory = ctx.attr.crate.label.workspace_root
    if ctx.attr.crate.label.package:
        package_directory += "/" + ctx.attr.crate.label.package
    license_files = [
        file
        for file in ctx.attr.crate[CrateInfo].compile_data.to_list()
        if file.dirname == package_directory and
           file.basename.upper().startswith("LICENSE")
    ]
    if not license_files:
        fail("{} has no recognized license file".format(ctx.attr.crate.label))
    license_file = license_files[0]

    return [InstallInfo(
        install_actions = [struct(
            src = license_file,
            dst = ctx.attr.doc_dest + "/" + license_file.basename,
        )],
        rename = {},
        installed_files = {},
    )]

install_crate_license = rule(
    implementation = _install_crate_license_impl,
    attrs = {
        "crate": attr.label(mandatory = True, providers = [CrateInfo]),
        "doc_dest": attr.string(mandatory = True),
    },
)
