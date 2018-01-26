# -*- python -*-

def _drake_exec_impl(ctx):
    # See doc below for `drake_exec`.
    # Using `$(location ...)` expansion for generated scripts gives crappy
    # non-runfiles-friendly paths.
    # `short_path` semi-solves it, but for externals, sometimes resolves to
    # `../*` instead of `external/*`
    target_relpath = ctx.executable.target.short_path
    if target_relpath.startswith('../'):
        target_relpath = "external/" + target_relpath[3:]
    info = dict(
        target_relpath = target_relpath,
        workspace_name = ctx.workspace_name,
    )
    content = """#!/bin/bash
set -e -u
# Ensure we have the correct runfiles directory.
runfiles_suffix=".runfiles/{workspace_name}"
runfiles_dir=$PWD
if [[ ${{runfiles_dir}} != *${{runfiles_suffix}} ]]; then
    script_path="$(cd $(dirname $0) && pwd)/$(basename $0)"
    runfiles_dir="${{script_path}}${{runfiles_suffix}}"
    [[ -d ${{runfiles_dir}} ]]
fi

# Invoke downstream target, letting it be aware of runfiles.
export BAZEL_RUNFILES=${{runfiles_dir}}
target_path=${{runfiles_dir}}/"{target_relpath}"
exec "${{target_path}}" "$@"
""".format(**info)
    ctx.file_action(
        output = ctx.outputs.executable,
        content = content,
        executable = True,
    )
    return [DefaultInfo(
        runfiles = ctx.runfiles(
            # Inherit `target`s runfiles.
            files = list(ctx.attr.target.data_runfiles.files),
        ),
    )]

"""
Enables a script to be run via `bazel run` or `bazel-bin` by ensuring that it
is aware of runfiles by the environment variable `BAZEL_RUNFILES`.
"""

drake_exec = rule(
    attrs = {
        "target": attr.label(
            cfg = "target",
            executable = True,
        ),
    },
    executable = True,
    implementation = _drake_exec_impl,
)
