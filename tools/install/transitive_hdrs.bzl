# -*- python -*-

# Collects the transitive closure of header files from ctx.attr.deps.
# This has headers from all (depended-on) externals, but not system headers.
def _transitive_hdrs_impl(ctx):
    headers = depset()
    for dep in ctx.attr.deps:
        headers += dep.cc.transitive_headers
    return struct(files = headers)

_transitive_hdrs = rule(
    attrs = {
        "deps": attr.label_list(
            allow_files = False,
            providers = ["cc"],
        ),
    },
    implementation = _transitive_hdrs_impl,
)

def transitive_hdrs_library(name, deps = [], **kwargs):
    """Declare a drake_cc_library that provides only hdrs (no srcs), where the
    hdrs are the hdrs for all cc_library targets named by "deps", transitively
    for all deps-of-deps, etc.

    All other kwargs are passed through to drake_cc_library.

    """

    _transitive_hdrs(
        name = name + "_gather",
        deps = deps,
        visibility = []
    )
    native.cc_library(
        name = name,
        hdrs = [":" + name + "_gather"],
        **kwargs
    )
