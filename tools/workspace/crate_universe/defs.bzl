load("@crates//:defs.bzl", _all_crate_deps = "all_crate_deps")
load("@rules_rust//rust/private:providers.bzl", "CrateInfo")
load("//tools/install:check_licenses.bzl", "LICENSE_LITERALS", "LICENSE_PREFIXES")
load("//tools/install:install.bzl", "InstallInfo")

def all_crate_deps():
    """Returns the fully qualified labels of all crate_universe crates."""
    return _all_crate_deps(package_name = "tools/workspace/crate_universe/lock")

def _install_crate_license_impl(ctx):
    files = ctx.attr.crate[CrateInfo].compile_data.to_list()
    package_directory = ctx.attr.crate.label.workspace_root
    if ctx.attr.crate.label.package:
        package_directory += "/" + ctx.attr.crate.label.package
    license_file = None
    preferred_basenames = ["LICENSE-APACHE", "LICENSE", "LICENSE.md", "LICENSE-MIT"]
    accepted_basenames = sorted([
        file.basename
        for file in files
        if file.dirname == package_directory and
           file.basename not in preferred_basenames and
           (file.basename in LICENSE_LITERALS or any([
               file.basename.startswith(prefix)
               for prefix in LICENSE_PREFIXES
           ]))
    ])
    for basename in preferred_basenames + accepted_basenames:
        matches = [
            file
            for file in files
            if file.dirname == package_directory and file.basename == basename
        ]
        if len(matches) > 1:
            fail("{} has multiple {} files".format(ctx.attr.crate.label, basename))
        if matches:
            license_file = matches[0]
            break

    if license_file == None:
        fail("{} has no recognized license file".format(ctx.attr.crate.label))

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
