# -*- python -*-

def _make_identifier(s):
    result = ""
    for i in range(len(s)):
        result += s[i] if s[i].isalnum() else "_"

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
        "#  define %s" % ctx.attr.no_export_macro_name,
        "#else",
        "#  define %s __attribute__((visibility(\"default\")))" % ctx.attr.export_macro_name,  # noqa
        "#  define %s __attribute__((visibility(\"hidden\")))" % ctx.attr.no_export_macro_name,  # noqa
        "#endif",
        "",
        "#ifndef %s" % ctx.attr.deprecated_macro_name,
        "#  define %s __attribute__ ((__deprecated__))" % ctx.attr.deprecated_macro_name,  # noqa
        "#endif",
        "",
        "#ifndef %s" % ctx.attr.export_deprecated_macro_name,
        "#  define %s %s %s" % (ctx.attr.export_deprecated_macro_name, ctx.attr.export_macro_name, ctx.attr.deprecated_macro_name),  # noqa
        "#endif",
        "",
        "#ifndef %s" % ctx.attr.no_export_deprecated_macro_name,
        "#  define %s %s %s" % (ctx.attr.no_export_deprecated_macro_name, ctx.attr.no_export_macro_name, ctx.attr.deprecated_macro_name),  # noqa
        "#endif",
        "",
        "#endif",
    ]

    ctx.actions.write(output = output, content = "\n".join(content) + "\n")

# Defines the rule to generate_export_header.
_generate_export_header_gen = rule(
    attrs = {
        "out": attr.output(mandatory = True),
        "export_macro_name": attr.string(),
        "deprecated_macro_name": attr.string(),
        "export_deprecated_macro_name": attr.string(),
        "no_export_macro_name": attr.string(),
        "no_export_deprecated_macro_name": attr.string(),
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
        deprecated_macro_name = None,
        export_deprecated_macro_name = None,
        no_export_macro_name = None,
        no_export_deprecated_macro_name = None,
        static_define = None,
        **kwargs):
    """Creates a rule to generate an export header for a named library.  This
    is an incomplete implementation of CMake's generate_export_header. (In
    particular, it assumes a platform that uses
    __attribute__((visibility("default"))) to decorate exports.)

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
    if deprecated_macro_name == None:
        deprecated_macro_name = "%s_DEPRECATED" % lib.upper()
    if export_deprecated_macro_name == None:
        export_deprecated_macro_name = "%s_DEPRECATED_EXPORT" % lib.upper()
    if no_export_macro_name == None:
        no_export_macro_name = "%s_NO_EXPORT" % lib.upper()
    if no_export_deprecated_macro_name == None:
        no_export_deprecated_macro_name = \
            "%s_DEPRECATED_NO_EXPORT" % lib.upper()
    if static_define == None:
        static_define = "%s_STATIC_DEFINE" % lib.upper()

    _generate_export_header_gen(
        name = name,
        out = out,
        export_macro_name = export_macro_name,
        deprecated_macro_name = deprecated_macro_name,
        export_deprecated_macro_name = export_deprecated_macro_name,
        no_export_macro_name = no_export_macro_name,
        no_export_deprecated_macro_name = no_export_deprecated_macro_name,
        static_define = static_define,
        **kwargs
    )
