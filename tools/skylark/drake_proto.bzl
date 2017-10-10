# -*- mode: python; -*- PYTHON-PREPROCESSING-REQUIRED

def _GetPath(ctx, path):
    if ctx.label.workspace_root:
        return ctx.label.workspace_root + '/' + path
    else:
        return path

def _GenDir(ctx):
    if not ctx.attr.includes:
        return ctx.label.workspace_root
    if not ctx.attr.includes[0]:
        return _GetPath(ctx, ctx.label.package)
    if not ctx.label.package:
        return _GetPath(ctx, ctx.attr.includes[0])
    return _GetPath(ctx, ctx.label.package + '/' + ctx.attr.includes[0])

def _CcOuts(srcs, use_grpc_plugin = False):
    ret = [s[:-len(".proto")] + ".pb.h" for s in srcs] + \
          [s[:-len(".proto")] + ".pb.cc" for s in srcs]
    if use_grpc_plugin:
        ret += [s[:-len(".proto")] + ".grpc.pb.h" for s in srcs] + \
               [s[:-len(".proto")] + ".grpc.pb.cc" for s in srcs]
    return ret

def _PyOuts(srcs):
    return [s[:-len(".proto")] + "_pb2.py" for s in srcs]

def _RelativeOutputPath(path, include, dest = ""):
    if include == None:
        return path

    if not path.startswith(include):
        fail("Include path %s isn't part of the path %s." % (include, path))

    if include and include[-1] != '/':
        include = include + '/'
    if dest and dest[-1] != '/':
        dest = dest + '/'

    path = path[len(include):]
    return dest + path

def _proto_gen_impl(ctx):
    """General implementation for generating protos"""
    srcs = ctx.files.srcs
    deps = []
    deps += ctx.files.srcs
    gen_dir = _GenDir(ctx)
    if gen_dir:
        gendirflag = "-I" + ctx.var["GENDIR"] + "/" + gen_dir
        import_flags = ["-I" + gen_dir, gendirflag]
    else:
        import_flags = ["-I."]

    for dep in ctx.attr.deps:
        import_flags += dep.proto.import_flags
        deps += dep.proto.deps

    args = []
    if ctx.attr.gen_cc:
        args += ["--cpp_out=" + ctx.var["GENDIR"] + "/" + gen_dir]
    if ctx.attr.gen_py:
        args += ["--python_out=" + ctx.var["GENDIR"] + "/" + gen_dir]

    if ctx.executable.plugin:
        plugin = ctx.executable.plugin
        lang = ctx.attr.plugin_language
        if not lang and plugin.basename.startswith('protoc-gen-'):
            lang = plugin.basename[len('protoc-gen-'):]
        if not lang:
            fail("cannot infer the target language of plugin",
                 "plugin_language")

        outdir = ctx.var["GENDIR"] + "/" + gen_dir
        if ctx.attr.plugin_options:
            outdir = ",".join(ctx.attr.plugin_options) + ":" + outdir
        args += ["--plugin=protoc-gen-%s=%s" % (lang, plugin.path)]
        args += ["--%s_out=%s" % (lang, outdir)]

    if args:
        paths = [s.path for s in srcs]
        cmdlist = ["$(which protoc)"] + args + import_flags + paths
        ctx.action(
            inputs = srcs + deps,
            outputs = ctx.outputs.outs,
            command = ' '.join(cmdlist),
            mnemonic = "ProtoCompile",
        )

    return struct(
        proto = struct(
            srcs = srcs,
            import_flags = import_flags,
            deps = deps,
        ),
    )

proto_gen = rule(
    attrs = {
        "srcs": attr.label_list(allow_files = True),
        "deps": attr.label_list(providers = ["proto"]),
        "includes": attr.string_list(),
        "plugin": attr.label(
            cfg = "host",
            allow_files = True,
            executable = True,
        ),
        "plugin_language": attr.string(),
        "plugin_options": attr.string_list(),
        "gen_cc": attr.bool(),
        "gen_py": attr.bool(),
        "outs": attr.output_list(),
    },
    output_to_genfiles = True,
    implementation = _proto_gen_impl,
)

"""Generates codes from Protocol Buffers definitions.

This rule helps you to implement Skylark macros specific to the target
language. You should prefer more specific `drake_cc_proto_library `,
`drake_py_proto_library` and others unless you are adding such wrapper macros.

Args:
  srcs: Protocol Buffers definition files (.proto) to run the protocol compiler
    against.
  deps: a list of dependency labels; must be other proto libraries.
  includes: a list of include paths to .proto files.
  plugin: the label of the protocol compiler plugin to be passed to the
    protocol compiler.
  plugin_language: the language of the generated sources
  plugin_options: a list of options to be passed to the plugin
  gen_cc: generates C++ sources in addition to the ones from the plugin.
  gen_py: generates Python sources in addition to the ones from the plugin.
  outs: a list of labels of the expected outputs from the protocol compiler.
"""

def drake_cc_proto_library(
        name,
        srcs = [],
        deps = [],
        cc_libs = [],
        include = None,
        internal_bootstrap_hack = False,
        use_grpc_plugin = False,
        **kargs):
    """Bazel rule to create a C++ protobuf library from proto source files

    NOTE: the rule is only an internal workaround to generate protos. The
    interface may change and the rule may be removed when bazel has introduced
    the native rule.

    Args:
      name: the name of the drake_cc_proto_library.
      srcs: the .proto files of the drake_cc_proto_library.
      deps: a list of dependency labels; must be drake_cc_proto_library.
      cc_libs: a list of other cc_library targets depended by the generated
          cc_library.
      include: a string indicating the include path of the .proto files.
      internal_bootstrap_hack: a flag indicate the drake_cc_proto_library is
          used only for bootstrapping. When it is set to True, no files will be
          generated. The rule will simply be a provider for .proto files, so
          that other drake_cc_proto_library can depend on it.
      use_grpc_plugin: a flag to indicate whether to call the grpc C++ plugin
          when processing the proto files.
          the generated cc_library target.
      **kargs: other keyword arguments that are passed to cc_library.

    """

    includes = []
    if include != None:
        includes = [include]

    if internal_bootstrap_hack:
        # For pre-checked-in generated files, we add the
        # internal_bootstrap_hack which will skip the codegen action.
        proto_gen(
            name = name + "_genproto",
            srcs = srcs,
            deps = [s + "_genproto" for s in deps],
            includes = includes,
            visibility = ["//visibility:public"],
        )
        # An empty cc_library to make rule dependency consistent.
        native.cc_library(
            name = name,
            **kargs)
        return

    grpc_cpp_plugin = None
    if use_grpc_plugin:
        grpc_cpp_plugin = "//external:grpc_cpp_plugin"

    outs = _CcOuts(srcs, use_grpc_plugin)

    proto_gen(
        name = name + "_genproto",
        srcs = srcs,
        deps = [s + "_genproto" for s in deps],
        includes = includes,
        plugin = grpc_cpp_plugin,
        plugin_language = "grpc",
        gen_cc = 1,
        outs = outs,
        visibility = ["//visibility:public"],
    )

    if use_grpc_plugin:
        cc_libs += ["//external:grpc_lib"]

    native.cc_library(
        name = name,
        srcs = outs,
        deps = cc_libs + deps + ["@systemprotobuf"],
        includes = includes,
        **kargs)

def internal_gen_well_known_protos_java(srcs):
    """Bazel rule to generate the gen_well_known_protos_java genrule

    Args:
      srcs: the well known protos
    """
    root = Label("%s//protobuf_java" % (REPOSITORY_NAME)).workspace_root
    if root == "":
        include = " -Isrc "
    else:
        include = " -I%s/src " % root
    native.genrule(
        name = "gen_well_known_protos_java",
        srcs = srcs,
        outs = [
            "wellknown.srcjar",
        ],
        cmd = "$(location :protoc) --java_out=$(@D)/wellknown.jar" +
              " %s $(SRCS) " % include +
              " && mv $(@D)/wellknown.jar $(@D)/wellknown.srcjar",
        tools = [":protoc"],
    )

def internal_copied_filegroup(name, srcs, strip_prefix, dest, **kwargs):
    """Macro to copy files to a different directory and then create a filegroup.

    This is used by the //:protobuf_python drake_py_proto_library target to
    work around an issue caused by Python source files that are part of the
    same Python package being in separate directories.

    Args:
      srcs: The source files to copy and add to the filegroup.
      strip_prefix: Path to the root of the files to copy.
      dest: The directory to copy the source files into.
      **kwargs: extra arguments that will be passesd to the filegroup.
    """
    outs = [_RelativeOutputPath(s, strip_prefix, dest) for s in srcs]

    native.genrule(
        name = name + "_genrule",
        srcs = srcs,
        outs = outs,
        cmd = " && ".join(
            ["cp $(location %s) $(location %s)" %
             (s, _RelativeOutputPath(s, strip_prefix, dest)) for s in srcs]),
    )

    native.filegroup(
        name = name,
        srcs = outs,
        **kwargs)

def drake_py_proto_library(
        name,
        srcs = [],
        deps = [],
        py_libs = [],
        py_extra_srcs = [],
        include = None,
        **kargs):
    """Bazel rule to create a Python protobuf library from proto source files

    NOTE: the rule is only an internal workaround to generate protos. The
    interface may change and the rule may be removed when bazel has introduced
    the native rule.

    Args:
      name: the name of the drake_py_proto_library.
      srcs: the .proto files of the drake_py_proto_library.
      deps: a list of dependency labels; must be drake_py_proto_library.
      py_libs: a list of other py_library targets depended by the generated
          py_library.
      py_extra_srcs: extra source files that will be added to the output
          py_library. This attribute is used for internal bootstrapping.
      include: a string indicating the include path of the .proto files.
      **kargs: other keyword arguments that are passed to cc_library.

    """
    outs = _PyOuts(srcs)

    includes = []
    if include != None:
        includes = [include]

    proto_gen(
        name = name + "_genproto",
        srcs = srcs,
        deps = [s + "_genproto" for s in deps],
        includes = includes,
        gen_py = 1,
        outs = outs,
        visibility = ["//visibility:public"],
    )

    native.py_library(
        name = name,
        srcs = outs + py_extra_srcs,
        deps = py_libs + deps,
        imports = includes,
        **kargs)
