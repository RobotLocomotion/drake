load(
    "//tools/skylark:python_env.bzl",
    "hermetic_python_env",
)

# Defines the implementation actions to cmake_configure_file.
def _cmake_configure_file_impl(ctx):
    if len(ctx.files.srcs) == 0:
        fail("There must be at least one srcs")
    if len(ctx.files.srcs) != len(ctx.outputs.outs):
        fail("The number of srcs and outs must be congruent")
    arguments = []
    for src in ctx.files.srcs:
        arguments += ["--input", src.path]
    for out in ctx.outputs.outs:
        arguments += ["--output", out.path]
    for item in ctx.attr.defines:
        arguments += ["-D" + item]
    for item in ctx.attr.undefines:
        arguments += ["-U" + item]
    for item in ctx.files.cmakelists:
        arguments += ["--cmakelists", item.path]
    if ctx.attr.autoconf:
        arguments += ["--autoconf"]
    if ctx.attr.strict:
        arguments += ["--strict"]
    if ctx.attr.atonly:
        arguments += ["--atonly"]
    ctx.actions.run(
        inputs = ctx.files.srcs + ctx.files.cmakelists,
        outputs = ctx.outputs.outs,
        arguments = arguments,
        env = ctx.attr.env,
        executable = ctx.executable.cmake_configure_file_py,
    )
    return []

# Defines the rule to cmake_configure_file.
_cmake_configure_file_gen = rule(
    attrs = {
        "srcs": attr.label_list(allow_files = True, mandatory = True),
        "outs": attr.output_list(mandatory = True),
        "defines": attr.string_list(),
        "undefines": attr.string_list(),
        "cmakelists": attr.label_list(allow_files = True),
        "autoconf": attr.bool(default = False),
        "strict": attr.bool(default = False),
        "atonly": attr.bool(default = False),
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
        strict = None,
        atonly = None,
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

    When strict is True, any substitution found in src that is not mentioned by
    either defines, undefines, or cmakelists is an error. When False, anything
    not mentioned is silently presumed to be undefined.

    When atonly is True, only substitutions like '@var@' will be made; '${var}'
    will be left as-is. When False, both types of substitutions will be made.

    See cmake_configure_file.py for our implementation of the configure_file
    substitution rules.

    The CMake documentation of the configure_file macro is:
    https://cmake.org/cmake/help/latest/command/configure_file.html

    """
    _cmake_configure_file_gen(
        name = name,
        srcs = [src],
        outs = [out],
        defines = defines,
        undefines = undefines,
        cmakelists = cmakelists,
        strict = strict,
        atonly = atonly,
        env = hermetic_python_env(),
        **kwargs
    )

def cmake_configure_files(
        name,
        srcs = None,
        outs = None,
        defines = None,
        undefines = None,
        cmakelists = None,
        strict = None,
        atonly = None,
        **kwargs):
    """Like cmake_configure_file(), but with itemwise pairs of srcs => outs,
    instead of just one pair of src => out.

    When in strict mode, the defines / undefines must be used by *at least one*
    of the srcs; only a definition that is unused by all srcs is an error.
    """
    _cmake_configure_file_gen(
        name = name,
        srcs = srcs,
        outs = outs,
        defines = defines,
        undefines = undefines,
        cmakelists = cmakelists,
        strict = strict,
        atonly = atonly,
        env = hermetic_python_env(),
        **kwargs
    )

def autoconf_configure_file(
        name,
        src = None,
        out = None,
        defines = None,
        undefines = None,
        strict = None,
        **kwargs):
    """Creates a rule to generate an out= file from a src= file, using autoconf
    substitution semantics.  This implementation is incomplete, and may not
    produce the same result as autoconf in all cases.

    Definitions are passed as defines= strings (with the usual convention of
    either a name-only "HAVE_FOO", or a key-value "MYSCALAR=DOUBLE").

    Variables that are known substitutions but which should be undefined can be
    passed as undefines= strings.

    When strict is True, any substitution found in src that is not mentioned by
    either defines or undefines is an error. When False, anything not mentioned
    is silently presumed to be undefined.
    """
    _cmake_configure_file_gen(
        name = name,
        srcs = [src],
        outs = [out],
        defines = defines,
        undefines = undefines,
        strict = strict,
        autoconf = True,
        env = hermetic_python_env(),
        **kwargs
    )
