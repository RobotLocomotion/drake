# -*- python -*-

MainClassInfo = provider()

# The CXX_FLAGS will be enabled for all C++ rules in the project
# building with any compiler.
CXX_FLAGS = [
    "-Werror=all",
    "-Werror=ignored-qualifiers",
    "-Werror=overloaded-virtual",
]

# The CLANG_FLAGS will be enabled for all C++ rules in the project when
# building with clang.
CLANG_FLAGS = CXX_FLAGS + [
    "-Werror=shadow",
    "-Werror=inconsistent-missing-override",
    "-Werror=sign-compare",
    "-Werror=return-stack-address",
    "-Werror=non-virtual-dtor",
]

# The GCC_FLAGS will be enabled for all C++ rules in the project when
# building with gcc.
GCC_FLAGS = CXX_FLAGS + [
    "-Werror=extra",
    "-Werror=return-local-addr",
    "-Werror=non-virtual-dtor",
    "-Werror=unused-but-set-parameter",
    # TODO(jwnimmer-tri) Fix these warnings and remove this suppression.
    "-Wno-missing-field-initializers",
    # TODO(#2852) Turn on shadow checking for g++ once we use a version that
    # fixes https://gcc.gnu.org/bugzilla/show_bug.cgi?id=57709
]

# The GCC_CC_TEST_FLAGS will be enabled for all cc_test rules in the project
# when building with gcc.
GCC_CC_TEST_FLAGS = [
    "-Wno-unused-parameter",
]

def _platform_copts(rule_copts, rule_gcc_copts, cc_test = 0):
    """Returns both the rule_copts (plus rule_gcc_copts iff under GCC), and
    platform-specific copts.

    When cc_test=1, the GCC_CC_TEST_FLAGS will be added.  It should only be set
    to 1 from cc_test rules or rules that are boil down to cc_test rules.
    """
    extra_gcc_flags = []
    if cc_test:
        extra_gcc_flags = GCC_CC_TEST_FLAGS
    return select({
        "//tools/cc_toolchain:gcc4.9-linux":
            GCC_FLAGS + extra_gcc_flags + rule_copts + rule_gcc_copts,
        "//tools/cc_toolchain:gcc5-linux":
            GCC_FLAGS + extra_gcc_flags + rule_copts + rule_gcc_copts,
        "//tools/cc_toolchain:clang3.9-linux": CLANG_FLAGS + rule_copts,
        "//tools/cc_toolchain:apple": CLANG_FLAGS + rule_copts,
        "//conditions:default": rule_copts,
    })

def _dsym_command(name):
    """Returns the command to produce .dSYM on OS X, or a no-op on Linux."""
    return select({
        "//tools/cc_toolchain:apple_debug":
            "dsymutil -f $(location :" + name + ") -o $@ 2> /dev/null",
        "//conditions:default": "touch $@",
    })

def _check_library_deps_blacklist(name, deps):
    """Report an error if a library should not use something from deps."""
    if not deps:
        return
    if type(deps) != 'list':
        # We can't handle select() yet.
        return
    for dep in deps:
        if dep.endswith(":main"):
            fail("The cc_library '" + name + "' must not depend on a :main " +
                 "function from a cc_library; only cc_binary program should " +
                 "have a main function")

# Generate a launcher file to run installed java binaries
def _drake_java_binary_install_launcher_impl(ctx):
    classpath = ctx.attr.target.java.compilation_info.runtime_classpath
    return [
        MainClassInfo(
            main_class = ctx.attr.main_class,
            classpath = classpath,
            filename = ctx.attr.filename,
        )
    ]

_drake_java_binary_install_launcher = rule(
    attrs = {
        "main_class": attr.string(mandatory = True),
        "filename": attr.string(mandatory = True),
        "target": attr.label(mandatory = True),
    },
    implementation = _drake_java_binary_install_launcher_impl,
)

"""Generate a launcher for java binary files.
"""

def drake_java_binary(
        name,
        main_class,
        **kwargs):
    """Creates a rule to declare a java binary and a MainClassInfo Provider

    The native java_binary creates a java launcher (shell script) that works in
    the build tree. However, a different launcher needs to be created to run
    the java binary in the install tree. This function generates a
    MainClassInfo provider that can be used by the installer (install.bzl) to
    configure the installation script that will copy the installed files and
    generate a launcher script at install time.
    """
    vkwargs = {
        key: value for key, value in kwargs.items()
        if key == "visibility"
    }
    native.java_binary(
        name = name,
        main_class = main_class,
        **kwargs)
    launcher_name = name + "-launcher"
    _drake_java_binary_install_launcher(
        name = launcher_name,
        main_class = main_class,
        target = ":" + name,
        filename = launcher_name + ".sh",
        **vkwargs)

def drake_cc_library(
        name,
        hdrs = None,
        srcs = None,
        deps = None,
        copts = [],
        gcc_copts = [],
        linkstatic = 1,
        **kwargs):
    """Creates a rule to declare a C++ library.

    By default, we produce only static libraries, to reduce compilation time
    on all platforms, and to avoid mysterious dyld errors on OS X. This default
    could be revisited if binary size becomes a concern.
    """
    _check_library_deps_blacklist(name, deps)
    native.cc_library(
        name = name,
        hdrs = hdrs,
        srcs = srcs,
        deps = deps,
        copts = _platform_copts(copts, gcc_copts),
        linkstatic = linkstatic,
        **kwargs)

def drake_cc_binary(
        name,
        hdrs = None,
        srcs = None,
        deps = None,
        copts = [],
        gcc_copts = [],
        linkstatic = 1,
        testonly = 0,
        add_test_rule = 0,
        test_rule_args = [],
        test_rule_size = None,
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
        name = name,
        hdrs = hdrs,
        srcs = srcs,
        deps = deps,
        copts = _platform_copts(copts, gcc_copts),
        testonly = testonly,
        linkstatic = linkstatic,
        **kwargs)

    # Also generate the OS X debug symbol file for this binary.
    native.genrule(
        name = name + "_dsym",
        srcs = [":" + name],
        outs = [name + ".dSYM"],
        output_to_bindir = 1,
        testonly = testonly,
        tags = ["dsym"],
        visibility = ["//visibility:private"],
        cmd = _dsym_command(name),
    )

    if "@gtest//:main" in (deps or []):
        fail("Use drake_cc_googletest to declare %s as a test" % name)

    if add_test_rule:
        drake_cc_test(
            name = name + "_test",
            hdrs = hdrs,
            srcs = srcs,
            deps = deps,
            copts = copts,
            size = test_rule_size,
            testonly = testonly,
            linkstatic = linkstatic,
            args = test_rule_args,
            **kwargs)

def drake_cc_test(
        name,
        size = None,
        srcs = None,
        copts = [],
        gcc_copts = [],
        disable_in_compilation_mode_dbg = False,
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
        srcs = select({
            "//tools/cc_toolchain:debug": [],
            "//conditions:default": srcs,
        })
    native.cc_test(
        name = name,
        size = size,
        srcs = srcs,
        copts = _platform_copts(copts, gcc_copts, cc_test = 1),
        **kwargs)

    # Also generate the OS X debug symbol file for this test.
    native.genrule(
        name = name + "_dsym",
        srcs = [":" + name],
        outs = [name + ".dSYM"],
        output_to_bindir = 1,
        testonly = 1,
        tags = ["dsym"],
        visibility = ["//visibility:private"],
        cmd = _dsym_command(name),
    )

def drake_cc_googletest(
        name,
        deps = None,
        use_default_main = True,
        **kwargs):
    """Creates a rule to declare a C++ unit test using googletest.

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.
    By default, sets use_default_main=True to use a default main() function.
    Otherwise, it will depend on @gtest//:without_main.

    If disable_in_compilation_mode_dbg is True, the srcs will be suppressed
    in debug-mode builds, so the test will trivially pass. This option should
    be used only rarely, and the reason should always be documented.
    """
    if deps == None:
        deps = []
    if use_default_main:
        deps += ["//drake/common/test_utilities:drake_cc_googletest_main"]
    else:
        deps += ["@gtest//:without_main"]
    drake_cc_test(
        name = name,
        deps = deps,
        **kwargs)

def drake_py_library(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    native.py_library(
        name = name,
        **kwargs)

def drake_py_binary(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    native.py_binary(
        name = name,
        **kwargs)

def drake_py_test(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    native.py_test(
        name = name,
        **kwargs)
