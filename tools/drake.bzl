# -*- python -*-

# The CLANG_FLAGS will be enabled for all C++ rules in the project when
# building with clang.
CLANG_FLAGS = [
    "-Werror=all",
    "-Werror=inconsistent-missing-override",
    "-Werror=sign-compare",
    "-Werror=non-virtual-dtor",
    "-Werror=return-stack-address",
]

# The GCC_FLAGS will be enabled for all C++ rules in the project when
# building with gcc.
GCC_FLAGS = [
    "-Werror=all",
    "-Werror=extra",
    "-Werror=return-local-addr",
    "-Werror=non-virtual-dtor",
    "-Wno-unused-parameter",
    "-Wno-missing-field-initializers",
]

def _platform_copts(rule_copts):
  """Returns both the rule_copts, and platform-specific copts."""
  return select({
      "//tools:gcc4.9-linux": GCC_FLAGS + rule_copts,
      "//tools:gcc5-linux": GCC_FLAGS + rule_copts,
      "//tools:clang3.9-linux": CLANG_FLAGS + rule_copts,
      "//tools:apple": CLANG_FLAGS + rule_copts,
      "//conditions:default": rule_copts,
  })

def _dsym_command(name):
  """Returns the command to produce .dSYM on OS X, or a no-op on Linux."""
  return select({
      "//tools:apple_debug":
          "dsymutil -f $(location :" + name + ") -o $@ 2> /dev/null",
      "//conditions:default": "touch $@",
  })

def drake_cc_library(
        name,
        hdrs=None,
        srcs=None,
        deps=None,
        copts=[],
        linkstatic=1,
        **kwargs):
    """Creates a rule to declare a C++ library.

    By default, we produce only static libraries, to reduce compilation time
    on all platforms, and to avoid mysterious dyld errors on OS X. This default
    could be revisited if binary size becomes a concern.
    """
    native.cc_library(
        name=name,
        hdrs=hdrs,
        srcs=srcs,
        deps=deps,
        copts=_platform_copts(copts),
        linkstatic=linkstatic,
        **kwargs)

def drake_cc_binary(
        name,
        hdrs=None,
        srcs=None,
        deps=None,
        copts=[],
        linkstatic=1,
        testonly=0,
        add_test_rule=0,
        test_rule_args=[],
        test_rule_size=None,
        **kwargs):
    """Creates a rule to declare a C++ binary.

    By default, we prefer to link static libraries whenever they are available.
    This default could be revisited if binary size becomes a concern.

    If you wish to create a smoke-test demonstrating that your binary runs
    without crashing, supply add_test_rule=1. Note that if you wish to do
    this, you should consider suppressing that urge, and instead writing real
    tests. The smoke-test will be named <name>_test. You may override cc_test
    defaults using test_rule_args=["-f", "--bar=42"] or test_rule_size="baz".
    """
    native.cc_binary(
        name=name,
        hdrs=hdrs,
        srcs=srcs,
        deps=deps,
        copts=_platform_copts(copts),
        testonly=testonly,
        linkstatic=linkstatic,
        **kwargs)

    # Also generate the OS X debug symbol file for this binary.
    native.genrule(
        name=name + "_dsym",
        srcs=[":" + name],
        outs=[name + ".dSYM"],
        output_to_bindir=1,
        testonly=testonly,
        tags=["dsym"],
        visibility=["//visibility:private"],
        cmd=_dsym_command(name),
    )

    if add_test_rule:
        drake_cc_test(
            name=name + "_test",
            hdrs=hdrs,
            srcs=srcs,
            deps=deps,
            copts=copts,
            size=test_rule_size,
            testonly=testonly,
            linkstatic=linkstatic,
            args=test_rule_args,
            **kwargs)

def drake_cc_test(
        name,
        size=None,
        srcs=None,
        copts=[],
        disable_in_compilation_mode_dbg=False,
        **kwargs):
    """Creates a rule to declare a C++ unit test.  Note that for almost all
    cases, drake_cc_googletest should be used, instead of this rule.

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.

    If disable_in_compilation_mode_dbg is True, the srcs will be suppressed
    in debug-mode builds, so the test will trivially pass. This option should
    be used only rarely, and the reason should always be documented.
    """
    if size == None:
        size = "small"
    if srcs == None:
        srcs = ["test/%s.cc" % name]
    if disable_in_compilation_mode_dbg:
        # Remove the test declarations from the test in debug mode.
        # TODO(david-german-tri): Actually suppress the test rule.
        srcs = select({"//tools:debug" : [], "//conditions:default" : srcs})
    native.cc_test(
        name=name,
        size=size,
        srcs=srcs,
        copts=_platform_copts(copts),
        **kwargs)

    # Also generate the OS X debug symbol file for this test.
    native.genrule(
        name=name + "_dsym",
        srcs=[":" + name],
        outs=[name + ".dSYM"],
        output_to_bindir=1,
        testonly=1,
        tags=["dsym"],
        visibility=["//visibility:private"],
        cmd=_dsym_command(name),
    )

def drake_cc_googletest(
        name,
        deps=None,
        **kwargs):
    """Creates a rule to declare a C++ unit test using googletest.  Always adds
    a deps= entry for googletest main (@gtest//:main).

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.

    If disable_in_compilation_mode_dbg is True, the srcs will be suppressed
    in debug-mode builds, so the test will trivially pass. This option should
    be used only rarely, and the reason should always be documented.
    """
    if deps == None:
        deps = []
    deps.append("@gtest//:main")
    drake_cc_test(
        name=name,
        deps=deps,
        **kwargs)

# Collects the transitive closure of header files from ctx.attr.deps.
def _transitive_hdrs_impl(ctx):
  headers = set()
  for dep in ctx.attr.deps:
    headers += dep.cc.transitive_headers
  return struct(files=headers)

_transitive_hdrs = rule(
    attrs = {
        "deps": attr.label_list(
            allow_files = False,
            providers = ["cc"],
        ),
    },
    implementation = _transitive_hdrs_impl,
)

load("@bazel_tools//tools/build_defs/pkg:pkg.bzl", "pkg_tar")

def drake_header_tar(name, deps=[], **kwargs):
  """Creates a .tar.gz that includes all the headers exported by the deps."""
  # TODO(david-german-tri): The --flagfile that Bazel generates to drive `tar`
  # tacks a spurious `..` onto the paths of external headers, which we then
  # have to clean up in package_drake.sh. It's not clear whether this is a
  # Bazel bug, or a bug in these macros.
  _transitive_hdrs(name=name + "_gather",
                   deps=deps)
  # We must specify a non-default strip prefix so that the tarball contains
  # relative and not absolute paths.
  pkg_tar(name=name,
          extension="tar.gz",
          files=[":" + name + "_gather"],
          strip_prefix="/")
