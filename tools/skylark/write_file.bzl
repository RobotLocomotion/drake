# -*- python -*-

def _write_file_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, ctx.attr.content, ctx.attr.is_executable)
    return [DefaultInfo(
        files = depset([out]),
    )]

"""Writes a file as a simple macro."""

write_file = rule(
    attrs = {
        "content": attr.string(mandatory = True),
        "is_executable": attr.bool(default = False),
    },
    output_to_genfiles = True,
    implementation = _write_file_impl,
)

def write_cmake_vars(name, vars):
    """Writes a dictionary of values as CMake variables."""
    content = ""
    for key, value in vars.items():
        content += 'set({key} "{value}" CACHE STRING "" FORCE)\n'.format(
            key = key, value = value)
    write_file(
        name = name,
        content = content,
    )
