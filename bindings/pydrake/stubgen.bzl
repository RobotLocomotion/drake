# -*- python -*-

def _impl(ctx):
    ruledir = ctx.genfiles_dir.path + "/" + ctx.label.package
    ctx.actions.run(
        mnemonic = "GenerateMypyStubs",
        executable = ctx.executable.tool,
        arguments = [
            "--quiet",
            "--package=" + ctx.attr.package,
            "--output=" + ruledir,
        ],
        outputs = ctx.outputs.outs,
    )
    return [DefaultInfo(
        files = depset(ctx.outputs.outs),
        data_runfiles = ctx.runfiles(files = ctx.outputs.outs),
    )]

generate_python_stubs = rule(
    implementation = _impl,
    attrs = {
        "tool": attr.label(
            mandatory = True,
            executable = True,
            # We use "target" config so that we will use the to-be-installed
            # pydrake binaries in order to populate the pyi stubs.
            cfg = "target",
        ),
        "package": attr.string(mandatory = True),
        "outs": attr.output_list(mandatory = True),
    },
)
