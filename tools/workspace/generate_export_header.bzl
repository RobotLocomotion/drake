# -*- python -*-

def _make_identifier(s):
    result = ""
    for c in s:
        result += c if c.isalnum() else "_"

    return result

# Defines the implementation actions to generate_export_header.
def _generate_export_header_impl(ctx):
    output = ctx.outputs.out

    guard = _make_identifier(output.basename.upper())

    content = [
        "#ifndef %s" % guard,
        "#define %s" % guard,
        "",
        "#ifdef %s" % ctx.attr.static_define,
        "#  define %s" % ctx.attr.export_macro_name,
        "#else",
        "#  define %s __attribute__((visibility(\"default\")))" % ctx.attr.export_macro_name,  # noqa
        "#endif",
        "",
        "#endif",
    ]

    ctx.file_action(output = output, content = "\n".join(content) + "\n")

# Defines the rule to generate_export_header.
_generate_export_header_gen = rule(
    attrs = {
        "out": attr.output(mandatory = True),
        "export_macro_name": attr.string(),
        "static_define": attr.string(),
    },
    output_to_genfiles = True,
    implementation = _generate_export_header_impl,
)

def generate_export_header(
        lib = None,
        name = None,
        out = None,
        export_macro_name = None,
        static_define = None,
        **kwargs):
    """Creates a rule to generate an export header for a named library.  This
    is an incomplete implementation of CMake's generate_export_header. (In
    particular, it does not generate NO_EXPORT or DEPRECATED symbols, and it
    assumes a platform that uses __attribute__((visibility("default"))) to
    decorate exports.)

    By default, the rule will have a mangled name related to the library name,
    and will produce "<lib>_export.h".

    The CMake documentation of the generate_export_header macro is:
    https://cmake.org/cmake/help/latest/module/GenerateExportHeader.html

    """

    if name == None:
        name = "__%s_export_h" % lib
    if out == None:
        out = "%s_export.h" % lib
    if export_macro_name == None:
        export_macro_name = "%s_EXPORT" % lib.upper()

    _generate_export_header_gen(
        name = name,
        out = out,
        export_macro_name = export_macro_name,
        static_define = static_define,
        **kwargs)
