# -*- python -*-

# The CLANG_FLAGS will be enabled for all C++ rules in the project when
# building with clang.
CLANG_FLAGS = [
    "-Werror=all",
    "-Werror=inconsistent-missing-override",
    "-Werror=sign-compare",
    "-Werror=return-stack-address",
]

# The GCC_FLAGS will be enabled for all C++ rules in the project when
# building with gcc.
GCC_FLAGS = [
    "-Werror=all",
    "-Werror=extra",
    "-Werror=return-local-addr",
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

def drake_cc_library(
        name,
        hdrs=None,
        srcs=None,
        deps=None,
        copts=[],
        **kwargs):
    """Creates a rule to declare a C++ library."""
    native.cc_library(
        name=name,
        hdrs=hdrs,
        srcs=srcs,
        deps=deps,
        copts=_platform_copts(copts),
        **kwargs)

def drake_cc_binary(
        name,
        hdrs=None,
        srcs=None,
        deps=None,
        copts=[],
        **kwargs):
    """Creates a rule to declare a C++ binary."""
    native.cc_binary(
        name=name,
        hdrs=hdrs,
        srcs=srcs,
        deps=deps,
        copts=_platform_copts(copts),
        **kwargs)

def drake_cc_googletest(
        name,
        size=None,
        srcs=None,
        deps=None,
        disable_in_compilation_mode_dbg=False,
        **kwargs):
    """Creates a rule to declare a C++ unit test using googletest.  Always adds a
    deps= entry for googletest main (@gtest//:main).

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
    if deps == None:
        deps = []
    deps.append("@gtest//:main")
    native.cc_test(
        name=name,
        size=size,
        srcs=srcs,
        deps=deps,
        **kwargs)
