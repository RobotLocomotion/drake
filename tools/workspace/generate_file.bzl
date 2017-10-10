# -*- python -*-

def _generate_file_impl(ctx):
    ctx.file_action(output = ctx.outputs.out, content = ctx.attr.content)

generate_file = rule(
    attrs = {
        "content": attr.string(),
        "out": attr.output(mandatory = True),
    },
    output_to_genfiles = True,
    implementation = _generate_file_impl,
)

"""Generate a file with specified content.

This creates a rule to generate a file with specified content (which is either
static or has been previously computed).

Args:
    content (:obj:`str`): Desired content of the generated file.
"""
