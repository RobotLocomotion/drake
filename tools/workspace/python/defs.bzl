load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("//tools/skylark:cc.bzl", "CcInfo")

_PY_CC_TOOLCHAIN_TYPE = "@rules_python//python/cc:toolchain_type"

# These rules are intended for use only by our neighboring BUILD.bazel file.
# The comments in that file explain how and why to use these rules' output.

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
