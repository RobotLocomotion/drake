load("@rules_cc//cc:defs.bzl", "cc_common")
load("//tools/skylark:cc.bzl", "CcInfo")

_PY_CC_TOOLCHAIN_TYPE = "@rules_python//python/cc:toolchain_type"

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
    doc = """Caclulates the linker flags for how to link a C/C++ exectuable
    that embeds a Python interpreter (e.g., for unit testing), based on the
    current Python toolchain.""",
)
