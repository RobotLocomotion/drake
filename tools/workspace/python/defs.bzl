load("@rules_cc//cc/common:cc_common.bzl", "cc_common")
load("//tools/skylark:cc.bzl", "CcInfo")

# These rules are intended for use only by our neighboring BUILD.bazel file.
# The comments in that file explain how and why to use these rules' output.

_PY_CC_TOOLCHAIN_TYPE = "@rules_python//python/cc:toolchain_type"

def _python_version(ctx):
    """Returns a string a containing the major.minor version number of the
    current Python toolchain."""
    py_cc_toolchain = ctx.toolchains[_PY_CC_TOOLCHAIN_TYPE].py_cc_toolchain
    version = py_cc_toolchain.python_version
    major_minor = version.split(".")[:2]
    return ".".join(major_minor)

def _python_version_txt_impl(ctx):
    """Implementation of the python_version_text() rule, below."""
    output = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(
        output = output,
        content = _python_version(ctx),
        is_executable = False,
    )
    return [DefaultInfo(
        files = depset([output]),
        data_runfiles = ctx.runfiles(files = [output]),
    )]

python_version_txt = rule(
    implementation = _python_version_txt_impl,
    toolchains = [_PY_CC_TOOLCHAIN_TYPE],
    doc = """Generates a text file containing the major.minor version number of
    the current Python toolchain, without any newlines.""",
)
