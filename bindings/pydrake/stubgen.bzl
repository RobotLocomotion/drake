def _impl(ctx):
    ctx.actions.run(
        mnemonic = "GenerateMypyStubs",
        executable = ctx.executable.tool,
        arguments = [x.path for x in ctx.outputs.outs],
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
        "outs": attr.output_list(mandatory = True),
    },
)
