# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/install:install.bzl", "InstallInfo")

def _impl(ctx):
    known_non_runfiles = [
        # These are installed in share/drake, but are not runfiles (at least,
        # not with these paths).
        "setup/Brewfile",
        "setup/install_prereqs",
        "setup/packages-bionic.txt",
        "setup/requirements.txt",
    ]
    drake_runfiles = []
    for dest in ctx.attr.target[InstallInfo].installed_files:
        prologue = "share/drake/"
        if not dest.startswith(prologue):
            continue
        drake_relative_path = dest[len(prologue):]
        if drake_relative_path in known_non_runfiles:
            continue
        drake_runfiles.append(drake_relative_path)
    content = {
        "runfiles": {
            "drake": sorted(drake_runfiles),
        },
    }
    ctx.actions.write(
        output = ctx.outputs.out,
        # TODO(jwnimmer-tri) The compact json format makes emacs sad; maybe we
        # should artisally write it out as python so it is more readable?
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
