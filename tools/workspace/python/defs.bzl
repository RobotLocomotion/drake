load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("//tools/skylark:cc.bzl", "CcInfo")

_PY_CC_TOOLCHAIN_TYPE = "@rules_python//python/cc:toolchain_type"

# These rules are intended for use only by our neighboring BUILD.bazel file.
# The comments in that file explain how and why to use these rules' output.

def _current_py_cc_libpython(ctx):
    py_cc_toolchain = ctx.toolchains[_PY_CC_TOOLCHAIN_TYPE].py_cc_toolchain
    linkopts = ["-lpython{}".format(py_cc_toolchain.python_version)]
    return [
        CcInfo(
            compilation_context = None,
            linking_context = cc_common.create_linking_context(
                linker_inputs = depset(
                    direct = [
                        cc_common.create_linker_input(
                            owner = ctx.label,
                            user_link_flags = depset(linkopts),
                        ),
                    ],
                ),
            ),
        ),
    ]

current_py_cc_libpython = rule(
    implementation = _current_py_cc_libpython,
    toolchains = [_PY_CC_TOOLCHAIN_TYPE],
    provides = [CcInfo],
    doc = """Provides the linker flags for how to link a C/C++ executable
    that embeds a Python interpreter (e.g., for unit testing), based on the
    current Python toolchain. Use this rule like a cc_library.""",
)

def _python_version_txt(ctx):
    py_cc_toolchain = ctx.toolchains[_PY_CC_TOOLCHAIN_TYPE].py_cc_toolchain
    output = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(
        output = output,
        content = py_cc_toolchain.python_version,
        is_executable = False,
    )
    return [DefaultInfo(
        files = depset([output]),
        data_runfiles = ctx.runfiles(files = [output]),
    )]

python_version_txt = rule(
    implementation = _python_version_txt,
    toolchains = [_PY_CC_TOOLCHAIN_TYPE],
    doc = """Generates a text file containing the major.minor version number of
    the current Python toolchain, without any newlines.""",
)
