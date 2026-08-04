load("//tools/skylark:cc.bzl", "CcInfo", "cc_common")

def _cc_defines_impl(ctx):
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    defines = deps_cc_infos.compilation_context.defines.to_list()
    if len(defines) > 0:
        value = ";".join(defines)
    else:
        value = "\"\""
    content = "set({} {})".format(ctx.attr.var, value)
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, content = content, is_executable = False)
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]

cc_defines = rule(
    implementation = _cc_defines_impl,
    doc = """
Provides access to the preprocessor definitions from the given dependencies.
""",
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
        "var": attr.string(),
    },
    fragments = ["cpp"],
)
