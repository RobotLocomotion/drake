# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

def _lcm_srcs_to_hdrs(lcm_package, lcm_srcs):
    result = []
    for item in lcm_srcs:
        if not item.endswith(".lcm"):
            fail(item + " does not end with .lcm")
        result.append(lcm_package + "/" + item[:-len(".lcm")] + ".hpp")
    return result

def _lcmhpp_impl(ctx):
    for lcm_src, output in zip(ctx.files.lcm_srcs, ctx.outputs.outs):
        ctx.action(
            inputs = [lcm_src],
            outputs = [output],
            arguments = [
                "--cpp", "--cpp-std=c++11",
                # TODO(jwnimmer-tri) Replace "6" with len(lcm_package).
                "--cpp-hpath=" + output.dirname[:-6],
                lcm_src.path],
            executable = ctx.executable.lcmgen,
            )
    return struct()

_lcmhpp_library_gen = rule(
    implementation=_lcmhpp_impl,
    attrs={
        "lcm_srcs": attr.label_list(allow_files = True),
        "lcmgen": attr.label(
            cfg = HOST_CFG, executable = True,
            default = Label("@lcm//:lcmgen")),
        "outs": attr.output_list(),
    },
    output_to_genfiles = True,
)

def lcm_cc_library(
        name,
        lcm_package=None,
        lcm_srcs=[],
        includes=None,
        deps=None,
        **kwargs):

    if lcm_package == None:
        fail("lcm_package must be provided")

    outs = _lcm_srcs_to_hdrs(lcm_package, lcm_srcs)
    _lcmhpp_library_gen(
        name=name + "_lcmhpp_library_gen",
        lcm_srcs=lcm_srcs,
        outs=outs)

    newdep = "@lcm//:lcm"
    deps = list(deps or [])
    if newdep not in deps:
        deps.append(newdep)

    newinclude = "."
    includes = list(includes or [])
    if newinclude not in includes:
        includes.append(newinclude)

    native.cc_library(
        name=name,
        hdrs=outs,
        deps=deps,
        includes=includes,
        **kwargs)
