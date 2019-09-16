# -*- python -*-

load(
    "@drake//tools/skylark:python_env.bzl",
    "hermetic_python_env",
)

# Defines the implementation actions to cmake_configure_file.
def _cmake_configure_file_impl(ctx):
    arguments = [
        "--input",
        ctx.file.src.path,
        "--output",
        ctx.outputs.out.path,
    ]
    for item in ctx.attr.defines:
        arguments += ["-D" + item]
    for item in ctx.attr.undefines:
        arguments += ["-U" + item]
    for item in ctx.files.cmakelists:
        arguments += ["--cmakelists", item.path]
    ctx.actions.run(
        inputs = [ctx.file.src] + ctx.files.cmakelists,
        outputs = [ctx.outputs.out],
        arguments = arguments,
        env = ctx.attr.env,
        executable = ctx.executable.cmake_configure_file_py,
    )
    return []

# Defines the rule to cmake_configure_file.
_cmake_configure_file_gen = rule(
    attrs = {
        "src": attr.label(
            allow_single_file = True,
            mandatory = True,
        ),
        "out": attr.output(mandatory = True),
        "defines": attr.string_list(),
        "undefines": attr.string_list(),
        "cmakelists": attr.label_list(allow_files = True),
        "cmake_configure_file_py": attr.label(
            cfg = "host",
            executable = True,
            default = Label("//tools/workspace:cmake_configure_file"),
        ),
        "env": attr.string_dict(
            mandatory = True,
            allow_empty = True,
        ),
    },
    output_to_genfiles = True,
    implementation = _cmake_configure_file_impl,
)

def cmake_configure_file(
        name,
        src = None,
        out = None,
        defines = None,
        undefines = None,
        cmakelists = None,
        **kwargs):
    """Creates a rule to generate an out= file from a src= file, using CMake's
    configure_file substitution semantics.  This implementation is incomplete,
    and may not produce the same result as CMake in all cases.

    Definitions optionally can be passed in directly as defines= strings (with
    the usual defines= convention of either a name-only "HAVE_FOO", or a
    key-value "MYSCALAR=DOUBLE").

    Definitions optionally can be read from simple CMakeLists files that
    contain statements of the form "set(FOO_MAJOR_VERSION 1)" and similar.

    Variables that are known substitutions but which should be undefined can be
    passed as undefines= strings.

    See cmake_configure_file.py for our implementation of the configure_file
    substitution rules.

    The CMake documentation of the configure_file macro is:
    https://cmake.org/cmake/help/latest/command/configure_file.html

    """
    _cmake_configure_file_gen(
        name = name,
        src = src,
        out = out,
        defines = defines,
        undefines = undefines,
        cmakelists = cmakelists,
        env = hermetic_python_env(),
        **kwargs
    )
