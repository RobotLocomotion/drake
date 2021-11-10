# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/install:install.bzl", "InstallInfo")
load("@drake//tools/skylark:pathutils.bzl", "basename")
load("@python//:version.bzl", "PYTHON_SITE_PACKAGES_RELPATH")

def _impl(ctx):
    known_non_runfiles = [
        # These are installed in share/drake, but are not runfiles (at least,
        # not with these paths).
        "manipulation/models/iiwa_description/iiwa_stack.LICENSE.txt",
        "setup/Brewfile",
        "setup/install_prereqs",
        "setup/packages-bionic.txt",
        "setup/packages-focal.txt",
        "setup/requirements.txt",
    ]
    known_non_runfiles_basenames = [
        "LICENSE",
        "LICENSE.TXT",
        "LICENSE.txt",
    ]
    drake_runfiles = []
    drake_prologue = "share/drake/"
    lcmtypes_drake_py_files = []
    lcmtypes_drake_py_prologue = PYTHON_SITE_PACKAGES_RELPATH + "/drake/"
    for dest in ctx.attr.target[InstallInfo].installed_files:
        if dest.startswith(drake_prologue):
            relative_path = dest[len(drake_prologue):]
            if relative_path in known_non_runfiles:
                continue
            if basename(relative_path) in known_non_runfiles_basenames:
                continue
            drake_runfiles.append(relative_path)
        elif dest.startswith(lcmtypes_drake_py_prologue):
            relative_path = dest[len(lcmtypes_drake_py_prologue):]
            lcmtypes_drake_py_files.append(relative_path)
    content = {
        "runfiles": {
            "drake": sorted(drake_runfiles),
        },
        "lcmtypes_drake_py": sorted(lcmtypes_drake_py_files),
        "python_site_packages_relpath": PYTHON_SITE_PACKAGES_RELPATH,
    }
    ctx.actions.write(
        output = ctx.outputs.out,
        content = "MANIFEST = " + struct(**content).to_json(),
        is_executable = False,
    )

generate_installed_files_manifest = rule(
    implementation = _impl,
    attrs = {
        "target": attr.label(
            mandatory = True,
            providers = [InstallInfo],
            doc = "The install target whose installed paths we'll enumerate.",
        ),
        "out": attr.output(
            mandatory = True,
            doc = "The bzl filename to write out.",
        ),
    },
)
"""Creates a manifest.bzl file containing the list of runfiles installed as
part of the given target.  Currently, only lists Drake's runfiles.
"""
