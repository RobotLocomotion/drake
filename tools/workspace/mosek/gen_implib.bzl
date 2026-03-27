_PY_CC_TOOLCHAIN_TYPE = "@rules_python//python/cc:toolchain_type"

def _impl(ctx):
    args = ctx.actions.args()
    args.add("--dlopen-callback", ctx.attr.dlopen_callback)
    args.add("--outdir", ctx.outputs.outs[0].dirname)
    args.add(ctx.file.src.path)

    py_toolchain = ctx.toolchains[_PY_CC_TOOLCHAIN_TYPE]

    ctx.actions.run(
        mnemonic = "GenImplib",
        executable = ctx.executable.tool,
        arguments = [args],
        inputs = [ctx.file.src],
        outputs = ctx.outputs.outs,
        tools = [
            ctx.executable.tool,
            py_toolchain.py3_runtime.files,
        ]
    )

    return [DefaultInfo(
        files = depset(ctx.outputs.outs),
    )]

gen_implib = rule(
    implementation = _impl,
    attrs = {
        "src": attr.label(allow_single_file = True, mandatory = True),
        "tool": attr.label(
            mandatory = True,
            executable = True,
            cfg = "target",
        ),
        "dlopen_callback": attr.string(mandatory = True),
        "outs": attr.output_list(mandatory = True),
    },
    toolchains = ["@rules_python//python/cc:toolchain_type"]
)
