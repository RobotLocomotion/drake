"""Provides a single point of control for rules_cc inside Drake.

For all code built by Drake (both first-party code, and built-from-source
externals), we should be using this file's macros. Any BUILD file that uses
`cc_foo()` rules should load these macros instead of loading from @rules_cc
directly.
"""

load(
    "@rules_cc//cc:cc_static_library.bzl",
    _cc_static_library = "cc_static_library",
)
load(
    "@rules_cc//cc:defs.bzl",
    _CcInfo = "CcInfo",
    _cc_binary = "cc_binary",
    _cc_common = "cc_common",
    _cc_import = "cc_import",
    _cc_library = "cc_library",
    _cc_shared_library = "cc_shared_library",
    _cc_test = "cc_test",
    _objc_library = "objc_library",
)

def _cc_system_library_impl(ctx):
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    old_context = deps_cc_infos.compilation_context
    new_context = cc_common.create_compilation_context(
        # We alter these fields.
        quote_includes = depset(),
        includes = depset(),
        system_includes = depset(
            transitive = [
                old_context.quote_includes,
                old_context.includes,
                old_context.system_includes,
            ],
        ),
        framework_includes = None,
        # We pass along all other fields unchanged. Sadly, there is no iterator.
        headers = old_context.headers,
        defines = old_context.defines,
        local_defines = old_context.local_defines,
    )
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                collect_data = True,
                collect_default = True,
            ),
        ),
        _CcInfo(
            compilation_context = new_context,
            linking_context = deps_cc_infos.linking_context,
        ),
    ]

_cc_system_library = rule(
    implementation = _cc_system_library_impl,
    doc = """
Changes the libary to use -isystem instead of -I for its include paths, which
will suppress compiler warnings from its header files.
""",
    attrs = {
        "deps": attr.label_list(providers = [_CcInfo]),
    },
    fragments = ["cpp"],
)

def _cc_library_isystem(**kwargs):
    name = kwargs.pop("name")
    testonly = kwargs.pop("testonly", None)
    visibility = kwargs.pop("visibility", None)
    hidden_name = "_{}_without_isystem".format(name)
    _cc_library(
        name = hidden_name,
        visibility = ["//visibility:private"],
        **kwargs
    )
    _cc_system_library(
        name = name,
        deps = [":" + hidden_name],
        testonly = testonly,
        visibility = visibility,
    )

def cc_binary(**kwargs):
    _cc_binary(**kwargs)

def cc_import(**kwargs):
    _cc_import(**kwargs)

def cc_library(**kwargs):
    """Wraps cc_library from rules_cc, except if isystem=True then switches the
    include spelling from -I to -isystem.
    """
    if kwargs.pop("isystem", False):
        _cc_library_isystem(**kwargs)
    else:
        _cc_library(**kwargs)

def cc_shared_library(**kwargs):
    _cc_shared_library(**kwargs)

def cc_static_library(**kwargs):
    _cc_static_library(**kwargs)

def cc_test(**kwargs):
    _cc_test(**kwargs)

def objc_library(**kwargs):
    _objc_library(**kwargs)

CcInfo = _CcInfo
cc_common = _cc_common
