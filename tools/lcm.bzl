# -*- python -*-

load(
    "@drake//tools:generate_include_header.bzl",
    "drake_generate_include_header",
)
load("@drake//tools:pathutils.bzl", "basename", "dirname", "join_paths")

def _lcm_aggregate_hdr(
        lcm_package,
        name,
        aggregate_hdr,
        hdrs,
        suffix,
        strip_prefix):
    """Create a header file aggregating all the given header files, and
    return the aggregate header file name (or an empty list if no file
    is generated).

    """
    if aggregate_hdr and len(hdrs):
        if aggregate_hdr == "AUTO":
            aggregate_hdr = join_paths(dirname(hdrs[0]),
                                       "%s.%s" % (lcm_package, suffix))
        drake_generate_include_header(
            name = name + "_lcm_aggregate_header",
            hdrs = hdrs,
            out = aggregate_hdr,
            strip_prefix = strip_prefix)

        return [aggregate_hdr]

    return []

def _lcm_outs(lcm_srcs, lcm_package, lcm_structs, extension):
    """Return the list of lcm-gen output filenames (derived from the lcm_srcs,
    lcm_package, and lcm_struct parameters as documented in lcm_cc_library
    below).  The filenames will use the given extension.

    """
    # Find and remove the dirname and extension shared by all lcm_srcs.
    # For srcs in the current directory, the dirname will be empty.
    subdir = dirname(lcm_srcs[0])
    lcm_names = []
    for item in lcm_srcs:
        if dirname(item) != subdir:
            fail("%s subdirectory doesn't match %s" % (item, lcm_srcs[0]))
        if not item.endswith(".lcm"):
            fail(item + " doesn't end with .lcm")
        itemname = basename(item)[:-len(".lcm")]
        lcm_names.append(itemname)

    # Assemble the expected output paths, inferring struct names from what we
    # got in lcm_srcs, if necessary.
    if extension == ".h":
        h_outs = [
            join_paths(subdir, lcm_package + "_" + lcm_struct + extension)
            for lcm_struct in (lcm_structs or lcm_names)]
        c_outs = [
            join_paths(subdir, lcm_package + "_" + lcm_struct + ".c")
            for lcm_struct in (lcm_structs or lcm_names)]
        outs = struct(hdrs = h_outs, srcs = c_outs)

    else:
        outs = [
            join_paths(subdir, lcm_package, lcm_struct + extension)
            for lcm_struct in (lcm_structs or lcm_names)]

    # Some languages have extra metadata.
    (extension in [".h", ".hpp", ".py", ".java"]) or fail(extension)
    if extension == ".py":
        outs.append(join_paths(subdir, lcm_package, "__init__.py"))

    return outs

def _lcmgen_impl(ctx):
    """The implementation actions to invoke lcm-gen.

    The ctx parameter comes from Skylark:
    https://bazel.build/versions/master/docs/skylark/lib/ctx.html
    """
    # We are given ctx.outputs.outs, which is the full path and file name of
    # the generated file we want to create.  However, except for the C
    # language, the lcm-gen tool places its outputs into a subdirectory of the
    # path we ask for, based on the LCM message's package name.  To set the
    # correct path, we need to both remove the filename from outs (which we do
    # via ".dirname"), as well as the package-name-derived directory name
    # (which we do via slicing off striplen characters), including the '/'
    # right before it (thus the "+ 1" below).
    if ctx.attr.language == "c":
        outpath = ctx.outputs.outs[0].dirname
    else:
        striplen = len(ctx.attr.lcm_package) + 1
        outpath = ctx.outputs.outs[0].dirname[:-striplen]

    if ctx.attr.language == "c":
        arguments = ["--c", "--c-cpath=" + outpath, "--c-hpath=" + outpath]
    elif ctx.attr.language == "cc":
        arguments = ["--cpp", "--cpp-std=c++11", "--cpp-hpath=" + outpath]
    elif ctx.attr.language == "py":
        arguments = ["--python", "--ppath=" + outpath]
    elif ctx.attr.language == "java":
        arguments = ["--java", "--jpath=" + outpath]
    else:
        fail("Unknown language")
    ctx.action(
        inputs = ctx.files.lcm_srcs,
        outputs = ctx.outputs.outs,
        arguments = arguments + [
            lcm_src.path for lcm_src in ctx.files.lcm_srcs],
        executable = ctx.executable.lcmgen,
    )
    return struct()

# Create rule to invoke lcm-gen on some lcm_srcs.
# https://www.bazel.io/versions/master/docs/skylark/rules.html
_lcm_library_gen = rule(
    attrs = {
        "lcm_srcs": attr.label_list(allow_files = True),
        "lcm_package": attr.string(),
        "lcmgen": attr.label(
            cfg = "host",
            executable = True,
            default = Label("@lcm//:lcm-gen"),
        ),
        "outs": attr.output_list(),
        "language": attr.string(),
    },
    output_to_genfiles = True,
    implementation = _lcmgen_impl,
)

def lcm_cc_library(
        name,
        lcm_srcs = None,
        lcm_package = None,
        lcm_structs = None,
        aggregate_hdr = None,
        aggregate_hdr_strip_prefix = ["**/include/"],
        **kwargs):
    """Declares a cc_library on message classes generated from `*.lcm` files.

    The required lcm_srcs list parameter specifies the `*.lcm` source files.
    All lcm_srcs must reside in the same subdirectory.

    The required lcm_package string parameter must match the `package ...;`
    statement in all of the files in lcm_srcs.

    The lcm_structs list parameter is optional; if unset, it defaults to the
    basenames of the files given in lcm_srcs.  If the struct names within the
    lcm_srcs do not match the basenames, or if the lcm_srcs declare multiple
    structs per file, then the parameter is required and must list every
    `struct ...;` declared by lcm_srcs. The ``aggregate_hdr`` parameter gives
    the name of the aggregate header to generate (default is ``None`` which
    disables the generation of the aggregate header). The special value
    ``"AUTO"`` will use the ``lcm_package`` name in the same subdirectory as
    the other headers, in conformance with the behavior of
    ``lcmUtilities.cmake``. ``aggregate_hdr_strip_prefix`` may be used to
    specify a list of prefixes which are removed from the names of the
    generated headers when forming the ``#include`` statements in the generated
    aggregate header. (See also the ``strip_prefix`` option of
    :func:`generate_include_header`.)
    """
    if not lcm_srcs:
        fail("lcm_srcs is required")
    if not lcm_package:
        fail("lcm_package is required")

    outs = _lcm_outs(lcm_srcs, lcm_package, lcm_structs, ".hpp")
    _lcm_library_gen(
        name = name + "_lcm_library_gen",
        language = "cc",
        lcm_srcs = lcm_srcs,
        lcm_package = lcm_package,
        outs = outs)

    if aggregate_hdr:
        outs += _lcm_aggregate_hdr(
            lcm_package,
            name,
            aggregate_hdr,
            outs,
            "hpp",
            aggregate_hdr_strip_prefix)

    deps = depset(kwargs.pop('deps', [])) | ["@lcm"]
    includes = depset(kwargs.pop('includes', [])) | ["."]
    native.cc_library(
        name = name,
        hdrs = outs,
        deps = deps,
        includes = includes,
        **kwargs)

def lcm_c_library(
        name,
        lcm_srcs,
        lcm_package,
        lcm_structs = None,
        aggregate_hdr = "AUTO",
        aggregate_hdr_strip_prefix = ["**/include/"],
        includes = [],
        **kwargs):
    """Declares a cc_library on message C structs generated from ``*.lcm``
    files.

    The required ``lcm_srcs`` :obj:`list` parameter specifies the ``*.lcm``
    source files. All ``lcm_srcs`` must reside in the same subdirectory.

    The standard parameters (``lcm_srcs``, ``lcm_package``, ``lcm_structs``,
    ``aggregate_hdr_strip_prefix``) are documented in :func:`lcm_cc_library`.
    The ``aggregate_hdr`` parameter default value is ``AUTO``.
    """
    outs = _lcm_outs(lcm_srcs, lcm_package, lcm_structs, ".h")

    _lcm_library_gen(
        name = name + "_lcm_library_gen",
        language = "c",
        lcm_srcs = lcm_srcs,
        lcm_package = lcm_package,
        outs = outs.hdrs + outs.srcs)

    hdrs = outs.hdrs
    if aggregate_hdr:
        hdrs += _lcm_aggregate_hdr(
            lcm_package,
            name,
            aggregate_hdr,
            outs.hdrs,
            "h",
            aggregate_hdr_strip_prefix)

    deps = depset(kwargs.pop('deps', [])) | ["@lcm"]
    includes = depset(includes) | ["."]
    native.cc_library(
        name = name,
        srcs = outs.srcs,
        hdrs = hdrs,
        deps = deps,
        includes = includes,
        **kwargs)

def lcm_py_library(
        name,
        lcm_srcs = None,
        lcm_package = None,
        lcm_structs = None,
        **kwargs):
    """Declares a py_library on message classes generated from `*.lcm` files.

    The standard parameters (lcm_srcs, lcm_package, lcm_structs) are documented
    in lcm_cc_library.

    This library has an ${lcm_package}/__init__.py, which means that this macro
    should only be used once for a given lcm_package in a given subdirectory.
    (Bazel will fail-fast with a "duplicate file" error if this is violated.)
    """
    if not lcm_srcs:
        fail("lcm_srcs is required")
    if not lcm_package:
        fail("lcm_package is required")

    outs = _lcm_outs(lcm_srcs, lcm_package, lcm_structs, ".py")
    _lcm_library_gen(
        name = name + "_lcm_library_gen",
        language = "py",
        lcm_srcs = lcm_srcs,
        lcm_package = lcm_package,
        outs = outs)

    imports = depset(kwargs.pop('imports', [])) | ["."]
    native.py_library(
        name = name,
        srcs = outs,
        imports = imports,
        **kwargs)

def lcm_java_library(
        name,
        lcm_srcs = None,
        lcm_package = None,
        lcm_structs = None,
        **kwargs):
    """Declares a java_library on message classes generated from `*.lcm` files.

    The standard parameters (lcm_srcs, lcm_package, lcm_structs) are documented
    in lcm_cc_library.

    """
    if not lcm_srcs:
        fail("lcm_srcs is required")
    if not lcm_package:
        fail("lcm_package is required")

    outs = _lcm_outs(lcm_srcs, lcm_package, lcm_structs, ".java")
    _lcm_library_gen(
        name = name + "_lcm_library_gen",
        language = "java",
        lcm_srcs = lcm_srcs,
        lcm_package = lcm_package,
        outs = outs)

    deps = depset(kwargs.pop('deps', [])) | ["@lcm//:lcm-java"]
    native.java_library(
        name = name,
        srcs = outs,
        deps = deps,
        **kwargs)
