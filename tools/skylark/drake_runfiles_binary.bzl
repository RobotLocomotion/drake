# -*- python -*-

# TODO(eric.cousineau): Deprecate and remove this when our binaries gain the
# ability to use Rlocation from Python.

def _drake_runfiles_binary_impl(ctx):
    # See doc below for `drake_runfiles_binary`.
    # Using `$(location ...)` expansion for generated scripts gives crappy
    # non-runfiles-friendly paths.
    # `short_path` semi-solves it, but for externals, sometimes resolves to
    # `../*` instead of `external/*`
    target_relpath = ctx.executable.target.short_path
    if target_relpath.startswith("../"):
        target_relpath = "external/" + target_relpath[3:]
    info = dict(
        target_relpath = target_relpath,
        workspace_name = ctx.workspace_name,
    )
    content = """#!/bin/bash
set -e -u
# Ensure we have the correct runfiles directory.
runfiles_suffix=".runfiles/{workspace_name}"
runfiles_dir="${{PWD}}"
if [[ "${{runfiles_dir}}" != *${{runfiles_suffix}} ]]; then
    script_path="$(cd $(dirname "$0") && pwd)/$(basename "$0")"
    runfiles_dir="${{script_path}}${{runfiles_suffix}}"
    if [[ ! -d "${{runfiles_dir}}" ]]; then
        echo "Runfiles not found: ${{runfiles_dir}}" >&2
        exit 1
    fi
fi

# Invoke downstream target, letting it be aware of runfiles.
export DRAKE_BAZEL_RUNFILES="${{runfiles_dir}}"
target_path="${{runfiles_dir}}/{target_relpath}"
exec "${{target_path}}" "$@"
""".format(**info)
    ctx.actions.write(
        output = ctx.outputs.executable,
        content = content,
        is_executable = True,
    )
    return [DefaultInfo(
        runfiles = ctx.runfiles(
            # Inherit `target`s runfiles.
            files = ctx.attr.target.data_runfiles.files.to_list(),
        ),
    )]

"""
Enables a script to be run via `bazel run` or `bazel-bin` by ensuring that it
is aware of runfiles by the environment variable `DRAKE_BAZEL_RUNFILES`.
"""

drake_runfiles_binary = rule(
    attrs = {
        "target": attr.label(
            cfg = "target",
            executable = True,
            mandatory = True,
        ),
    },
    executable = True,
    implementation = _drake_runfiles_binary_impl,
)
