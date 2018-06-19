# -*- python -*-

"""Declares a repository that names the parent directory of the Drake
workspace.  This repository contains no rules; just a single file named
`path.bzl` which has a variable `additional_transitive_quote_include_directory`
with Drake's parent directory name.  This path is used by `aspect.bzl` in this
package (@drake//tools/clion).  This rule only makes sense when used directly
from Drake's WORKSPACE file, not from any other project; thus, it does not live
under //tools/workspace like most other repository rules.
"""

def _impl(repository_ctx):
    # Find the top-level folder of Drake's source code.
    dotfile_path = repository_ctx.path(Label("@drake//:.bazelproject"))
    drake_workspace = dotfile_path.realpath.dirname

    # Compute an include path that makes '#include "drake/foo.h" work.
    if drake_workspace.basename == "drake":
        drake_workspace_parent = str(drake_workspace.dirname)
    else:
        print("Cannot fix CLion paths; your checkout is not named 'drake'")
        drake_workspace_parent = ""

    # Write out the path to a bzl constant.
    bzl_content = [
        'additional_transitive_quote_include_directory = "{}"'.format(
            drake_workspace_parent,
        ),
    ]
    repository_ctx.file(
        "BUILD.bazel",
        content = "\n",
        executable = False,
    )
    repository_ctx.file(
        "path.bzl",
        content = "".join(bzl_content),
        executable = False,
    )

def drake_clion_environment():
    rule = repository_rule(
        implementation = _impl,
        local = True,
    )
    rule(name = "drake_clion_environment")
