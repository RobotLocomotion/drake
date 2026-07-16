load("//tools/skylark:cc.bzl", "CcInfo", "cc_common")

def _extract_nanobind_header_impl(ctx):
    result = None
    for dep in ctx.attr.deps:
        cc_info = dep[CcInfo]
        for header in cc_info.compilation_context.headers.to_list():
            if header.short_path.endswith(ctx.attr.header):
                result = header
                break
    if result == None:
        fail("No match")
    return [DefaultInfo(files = depset([result]))]

extract_nanobind_header = rule(
    implementation = _extract_nanobind_header_impl,
    attrs = {
        "header": attr.string(mandatory = True),
        "deps": attr.label_list(providers = [CcInfo], mandatory = True),
    },
)
