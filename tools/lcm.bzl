# -*- python -*-

# This file contains rules for the Bazel build system.
# See http://bazel.io/ .

# In particular, this is a Skylark extension:
# https://bazel.build/versions/master/docs/skylark/concepts.html

def _lcm_srcs_to_hdrs(lcm_package, lcm_srcs):
    """Return the list of filenames for a cc_library's hdrs= parameter, based on
    the lcm_package= and lcm_srcs= parameters (see lcm_cc_library below for a
    description of what those parameters mean).

    """
    result = []
    for item in lcm_srcs:
        if not item.endswith(".lcm"):
            fail(item + " does not end with .lcm")
        result.append(lcm_package + "/" + item[:-len(".lcm")] + ".hpp")
    return result

def _lcmhpp_impl(ctx):
    """The implementation actions to invoke lcmgen.

    The ctx parameter comes from Skylark:
    https://bazel.build/versions/master/docs/skylark/lib/ctx.html
    """
    # We are given ctx.outputs.outs, which is the full path and file name of
    # the generated file we want to create.  However, the lcmgen tool (in C++
    # mode) places its outputs into a subdirectory of its --cpp-hpath directory
    # name we tell it, based on the LCM message's package name.  To set the
    # correct --cpp-hpath value, we need to both remove the filename from outs
    # (which we do via ".dirname"), as well as the package-name-derived
    # directory name (which we do via slicing off striplen characters),
    # including the '/' right before it (thus the "+ 1" below).
    striplen = len(ctx.attr.lcm_package) + 1
    for lcm_src, output in zip(ctx.files.lcm_srcs, ctx.outputs.outs):
        ctx.action(
            inputs = [lcm_src],
            outputs = [output],
            arguments = [
                "--cpp", "--cpp-std=c++11",
                "--cpp-hpath=" + output.dirname[:-striplen],
                lcm_src.path],
            executable = ctx.executable.lcmgen,
            )
    return struct()

# Create rule to invoke lcmgen on some lcm_srcs.
# https://www.bazel.io/versions/master/docs/skylark/rules.html
_lcmhpp_library_gen = rule(
    implementation = _lcmhpp_impl,
    attrs = {
        "lcm_package": attr.string(),
        "lcm_srcs": attr.label_list(allow_files = True),
        "lcmgen": attr.label(
            cfg = "host", executable = True,
            default = Label("@lcm//:lcmgen")),
        "outs": attr.output_list(),
    },
    output_to_genfiles = True,
)

def lcm_cc_library(
        name,
        lcm_package=None,
        lcm_srcs=None,
        includes=None,
        deps=None,
        **kwargs):
    """Declares a cc_library target based on C++ message classes generated from
    `*.lcm` files.  The lcm_src= list parameter specifies the `*.lcm` sources.
    The lcm_package= string parameter must match the `package ...;` statement
    in all of the `*.lcm` files in lcm_srcs.  The Bazel-standard includes= and
    deps= parameters are passed through to cc_library after adding the items
    required by the generated code.

    """
    if lcm_package == None:
        fail("lcm_package must be provided")
    if not lcm_srcs:
        fail("lcm_srcs must be provided")

    # Note that only C++ codegen is supported for now.  If desired in the
    # future, we can add more parameters to our method that allow users to
    # select different languages.

    outs = _lcm_srcs_to_hdrs(lcm_package, lcm_srcs)
    _lcmhpp_library_gen(
        name=name + "_lcmhpp_library_gen",
        lcm_package=lcm_package,
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
