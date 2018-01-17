# -*- python -*-

load(
    "//tools/skylark:6996.bzl",
    "adjust_label_for_drake_hoist",
    "adjust_labels_for_drake_hoist",
)

# Keep CXX_FLAGS, CLANG_FLAGS, and GCC_FLAGS in sync with CMAKE_CXX_FLAGS in
# matlab/cmake/flags.cmake.

# The CXX_FLAGS will be enabled for all C++ rules in the project
# building with any compiler.
CXX_FLAGS = [
    "-Werror=all",
    "-Werror=ignored-qualifiers",
    "-Werror=overloaded-virtual",
    "-Werror=old-style-cast",
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
    "-Werror=logical-op",
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
        # TODO(jwnimmer-tri) We should handle select.
        return
    for dep in deps:
        if dep.endswith(":main"):
            fail("The cc_library '" + name + "' must not depend on a :main " +
                 "function from a cc_library; only cc_binary program should " +
                 "have a main function")

def _prune_private_hdrs(srcs):
    """Returns (new_srcs, private_hdrs), where .h files have been split out of
    srcs into private_hdrs, leaving new_srcs remaining.
    """
    if type(srcs) == "select":
        # We can't handle select() yet.
        # TODO(jwnimmer-tri) We should handle select.
        return srcs, []
    private_hdrs = [x for x in srcs if x.endswith(".h")]
    if private_hdrs:
        srcs = [x for x in srcs if x not in private_hdrs]
    return srcs, private_hdrs

def installed_headers_for_dep(dep):
    """Convert a cc_library label to a DrakeCc provider label.  Given a label
    `dep` for a cc_library, such as would be found in the the `deps = []` of
    some cc_library, returns the corresponding label for the matching DrakeCc
    provider associated with that library.  The returned label is appropriate
    to use in the deps of of a `drake_installed_headers()` rule.

    Once our rules are better able to call native rules like native.cc_binary,
    instead of having two labels we would prefer to tack a DrakeCc provider
    onto the cc_library target directly.

    Related links from upstream:
    https://github.com/bazelbuild/bazel/issues/2163
    https://docs.bazel.build/versions/master/skylark/cookbook.html#macro-multiple-rules
    """
    suffix = ".installed_headers"
    if ":" in dep:
        # The label is already fully spelled out; just tack on our suffix.
        result = dep + suffix
    else:
        # The label is the form //foo/bar which means //foo/bar:bar.
        last_slash = dep.rindex("/")
        libname = dep[last_slash + 1:]
        result = dep + ":" + libname + suffix
    return adjust_label_for_drake_hoist(result)

def installed_headers_for_drake_deps(deps):
    """Filters `deps` to find drake labels (i.e., discard third_party labels),
    and then maps `installed_headers_for_dep()` over that list of drake deps.

    (Absolute paths to Drake's lcmtypes headers are also filtered out, because
    LCM headers follow a different #include convention, and so are installed
    separately.  Refer to drake/lcmtypes/BUILD.bazel for details.  Note that
    within-package paths are left unchanged, so that this macro can still be
    used within Drake's lcmtypes folder.)

    This is useful for computing the deps of a `drake_installed_headers()` rule
    from the deps of a `cc_library()` rule.
    """
    if type(deps) == "select":
        # We can't handle select() yet.
        # TODO(jwnimmer-tri) We should handle select.
        return []
    return [
        installed_headers_for_dep(x)
        for x in deps if (
            not x.startswith("@") and
            not x.startswith("//drake/lcmtypes:")
        )
    ]

# A provider to collect Drake metadata about C++ rules.  For background, see
# https://docs.bazel.build/versions/master/skylark/rules.html#providers.
DrakeCc = provider()

def _drake_installed_headers_impl(ctx):
    hdrs = list(ctx.files.hdrs)
    solibs = list(ctx.files.solibs)
    for x in ctx.files.hdrs_exclude:
        hdrs.remove(x)
    transitive_hdrs = depset(hdrs)
    transitive_solibs = depset(solibs)
    for dep in ctx.attr.deps:
        transitive_hdrs += depset(dep[DrakeCc].transitive_hdrs)
        transitive_solibs += depset(dep[DrakeCc].transitive_solibs)
    return [
        DrakeCc(
            transitive_hdrs = transitive_hdrs,
            transitive_solibs = transitive_solibs,
        )
    ]

"""Declares a rule to provide DrakeCc information about headers that should be
installed.  We use this instead of the built-in `cc` provider so that we can
adjust and filter what is going to be installed, versus everything that is
required to compile.
"""

drake_installed_headers = rule(
    attrs = {
        "hdrs": attr.label_list(
            mandatory = True,
            allow_files = True,
        ),
        "hdrs_exclude": attr.label_list(
            allow_files = True,
        ),
        "solibs": attr.label_list(
            mandatory = False,
            allow_files = False,
        ),
        "deps": attr.label_list(
            mandatory = True,
            providers = [DrakeCc],
        ),
    },
    implementation = _drake_installed_headers_impl,
)

# TODO: Rename to `gather_transitive_artifacts`
def drake_transitive_installed_hdrs_filegroup(name, deps = [], **kwargs):
    """Declare a filegroup that contains the transtive installed hdrs of the
    targets named by `deps`.
    """
    drake_installed_headers(
        name = name,
        hdrs = [],
        deps = [installed_headers_for_dep(x) for x in deps],
        **kwargs
    )

def _drake_abslabel_to_solib_name(abslabel):
    if not abslabel.startswith("//"):
        fail("Internal error: {} should be absolute".format(abslabel))
    if abslabel.endswith(".so"):
        fail("Target should not already end in '.so': {}".format(abslabel))
    out = "libdrake-" + abslabel[2:].replace("/", "-").replace(":", "-") + ".so"
    return out

def _drake_cc_solib_name(target_name):
    if target_name.startswith("/"):
        fail("Internal error: {} ".format(target_name) +
             "should be a target name (not absolute label)")
    abslabel = "//" + native.package_name() + ":" + target_name
    return _drake_abslabel_to_solib_name(abslabel)

def _raw_drake_cc_library(
        name,
        hdrs = [],
        srcs = [],  # Cannot list any headers here.
        data = [],
        deps = [],
        solib_name = None,
        declare_installed_headers = 0,
        install_hdrs_exclude = [],
        testonly = None,
        linkshared = 1,
        linkstatic = 0,
        **kwargs):
    """Creates a rule to declare a C++ library.  Uses Drake's include_prefix and
    checks the deps blacklist.  If declare_installed_headers is true, also adds
    a drake_installed_headers() target.  (This should be set if and only if the
    caller is drake_cc_library.)
    """
    data = adjust_labels_for_drake_hoist(data)
    deps = adjust_labels_for_drake_hoist(deps)
    _check_library_deps_blacklist(name, deps)
    _, private_hdrs = _prune_private_hdrs(srcs)
    if private_hdrs:
        fail("private_hdrs = " + private_hdrs)
    if native.package_name().startswith("drake"):
        strip_include_prefix = None
        include_prefix = None
    else:
        # Require include paths like "drake/foo/bar.h", not "foo/bar.h".
        strip_include_prefix = "/"
        include_prefix = "drake"
    if linkshared:
        if solib_name == None:
            solib_name = _drake_cc_solib_name(name)
        hdr_lib, solib = cc_solib_library(
            name = name,
            solib_name = solib_name,
            hdrs = hdrs,
            srcs = srcs,
            deps = deps,
            data = data,
            linkshared = 1,
            linkstatic = linkstatic,
            strip_include_prefix = strip_include_prefix,
            include_prefix = include_prefix,
            testonly = testonly,
            **kwargs)
        solibs = [solib]
    else:
        if not linkstatic:
            fail("Non-shared libraries *must* link statically.")
        # Static linking.
        native.cc_library(
            name = name,
            hdrs = hdrs,
            deps = deps,
            data = data,
            linkstatic = 1,
            strip_include_prefix = strip_include_prefix,
            include_prefix = include_prefix,
            testonly = testonly,
            **kwargs)
        solibs = []
    if declare_installed_headers:
        drake_installed_headers(
            name = name + ".installed_headers",
            hdrs = hdrs,
            hdrs_exclude = install_hdrs_exclude,
            solibs = solibs,
            deps = installed_headers_for_drake_deps(deps),
            testonly = testonly,
            tags = ["nolint"],
            visibility = ["//visibility:public"],
        )

def _maybe_add_pruned_private_hdrs_dep(
        base_name,
        srcs,
        deps,
        **kwargs):
    """Given some srcs, prunes any header files into a separate cc_library, and
    appends that new library to deps, returning new_srcs (sans headers) and
    new_deps.  The separate cc_library is private with linkstatic = 1.

    We use this helper in all drake_cc_{library,binary,test) because when we
    want to fiddle with include paths, we *must* have all header files listed
    as hdrs; the include_prefix does not apply to srcs.
    """
    new_srcs, private_hdrs = _prune_private_hdrs(srcs)
    if private_hdrs:
        name = "_" + base_name + "_private_headers_impl"
        kwargs.pop('linkshared', '')
        kwargs.pop('linkstatic', '')
        kwargs.pop('visibility', '')
        _raw_drake_cc_library(
            name = name,
            hdrs = private_hdrs,
            srcs = [],
            deps = deps,
            visibility = ["//visibility:private"],
            # Header-only.
            linkstatic = 1,
            linkshared = 0,
            **kwargs)
        new_deps = deps + [":" + name]
    else:
        new_deps = deps
    return new_srcs, new_deps

def drake_cc_library(
        name,
        hdrs = [],
        srcs = [],
        deps = [],
        copts = [],
        solib_name = None,
        gcc_copts = [],
        linkstatic = 0,
        linkshared = 1,
        install_hdrs_exclude = [],
        **kwargs):
    """Creates a rule to declare a C++ library.

    By default, we produce only static libraries, to reduce compilation time
    on all platforms, and to avoid mysterious dyld errors on OS X. This default
    could be revisited if binary size becomes a concern.

    The deps= of a drake_cc_library must either be another drake_cc_library, or
    be named like "@something//etc..." (i.e., come from the workspace, not part
    of Drake).  In other words, all of Drake's C++ libraries must be declared
    using the drake_cc_library macro.
    """
    new_copts = _platform_copts(copts, gcc_copts)
    # We install private_hdrs by default, because Bazel's visibility denotes
    # whether headers can be *directly* included when using cc_library; it does
    # not precisely relate to which headers should appear in the install tree.
    # For example, common/symbolic.h is the only public-visibility header for
    # its cc_library, but we also need to install all of its child headers that
    # it includes, such as common/symbolic_expression.h.
    new_srcs, new_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = deps,
        copts = new_copts,
        declare_installed_headers = 1,
        **kwargs)
    _raw_drake_cc_library(
        name = name,
        solib_name = solib_name,
        hdrs = hdrs,
        srcs = new_srcs,
        deps = new_deps,
        copts = new_copts,
        linkshared = linkshared,
        linkstatic = linkstatic,
        declare_installed_headers = 1,
        install_hdrs_exclude = install_hdrs_exclude,
        **kwargs)

def drake_cc_binary(
        name,
        srcs = [],
        data = [],
        deps = [],
        copts = [],
        linkopts = [],
        gcc_copts = [],
        linkshared = 0,
        linkstatic = 0,
        testonly = 0,
        add_test_rule = 0,
        test_rule_args = [],
        test_rule_data = [],
        test_rule_size = None,
        test_rule_flaky = 0,
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
    data = adjust_labels_for_drake_hoist(data)
    deps = adjust_labels_for_drake_hoist(deps)
    new_copts = _platform_copts(copts, gcc_copts)
    new_srcs, new_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = deps,
        copts = new_copts,
        testonly = testonly,
        **kwargs)
    if linkshared == 1:
        # On Linux, we need to disable "new" dtags in the linker so that we use
        # RPATH instead of RUNPATH.  When doing runtime linking, RPATH is
        # checked *before* LD_LIBRARY_PATH, which is important to avoid using
        # the MATLAB versions of certain libraries (protobuf).  macOS doesn't
        # understand this flag, so it is conditional on Linux only.  Note that
        # the string we use for rpath here doesn't actually matter; it will be
        # replaced during installation later.
        linkopts = select({
            "//tools/cc_toolchain:apple": linkopts,
            "//conditions:default": linkopts + [
                "-Wl,-rpath=/usr/lib/x86_64-linux-gnu -Wl,--disable-new-dtags",
            ],
        })

    native.cc_binary(
        name = name,
        srcs = new_srcs,
        data = data,
        deps = new_deps,
        copts = new_copts,
        testonly = testonly,
        linkshared = linkshared,
        linkstatic = linkstatic,
        linkopts = linkopts,
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

    if "@gtest//:main" in deps:
        fail("Use drake_cc_googletest to declare %s as a test" % name)

    if add_test_rule:
        drake_cc_test(
            name = name + "_test",
            srcs = new_srcs,
            data = data + test_rule_data,
            deps = new_deps,
            copts = copts,
            gcc_copts = gcc_copts,
            size = test_rule_size,
            flaky = test_rule_flaky,
            linkstatic = linkstatic,
            args = test_rule_args,
            **kwargs)

def drake_cc_test(
        name,
        size = None,
        srcs = [],
        data = [],
        deps = [],
        copts = [],
        gcc_copts = [],
        disable_in_compilation_mode_dbg = False,
        **kwargs):
    """Creates a rule to declare a C++ unit test.  Note that for almost all
    cases, drake_cc_googletest should be used, instead of this rule.

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.
    Unconditionally forces testonly=1.

    If disable_in_compilation_mode_dbg is True, the srcs will be suppressed
    in debug-mode builds, so the test will trivially pass. This option should
    be used only rarely, and the reason should always be documented.
    """
    data = adjust_labels_for_drake_hoist(data)
    deps = adjust_labels_for_drake_hoist(deps)
    if size == None:
        size = "small"
    if not srcs:
        srcs = ["test/%s.cc" % name]
    kwargs['testonly'] = 1
    new_copts = _platform_copts(copts, gcc_copts, cc_test = 1)
    new_srcs, new_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = deps,
        copts = new_copts,
        **kwargs)
    if disable_in_compilation_mode_dbg:
        # Remove the test declarations from the test in debug mode.
        # TODO(david-german-tri): Actually suppress the test rule.
        new_srcs = select({
            "//tools/cc_toolchain:debug": [],
            "//conditions:default": new_srcs,
        })
    native.cc_test(
        name = name,
        size = size,
        srcs = new_srcs,
        data = data,
        deps = new_deps,
        copts = new_copts,
        **kwargs)

    # Also generate the OS X debug symbol file for this test.
    native.genrule(
        name = name + "_dsym",
        srcs = [":" + name],
        outs = [name + ".dSYM"],
        output_to_bindir = 1,
        testonly = kwargs['testonly'],
        tags = ["dsym"],
        visibility = ["//visibility:private"],
        cmd = _dsym_command(name),
    )

def drake_cc_googletest(
        name,
        deps = [],
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
    if use_default_main:
        deps = deps + adjust_labels_for_drake_hoist([
            "//drake/common/test_utilities:drake_cc_googletest_main",
        ])
    else:
        deps = deps + ["@gtest//:without_main"]
    drake_cc_test(
        name = name,
        deps = deps,
        **kwargs)

# TODO(eric.cousineau): Remove this once it's unneeded.
def drake_example_cc_binary(
        srcs = [],
        deps = [],
        **kwargs):
    """Creates a rule to declare a C++ binary using `libdrake.so`.

    This rule is a wrapper around `drake_cc_binary()`. It adds `libdrake.so`
    and `drake_lcmtypes_headers` as dependencies to the target.

    This allows the creation of examples for drake that depend on `libdrake.so`
    which let the process discover the location of drake resources at runtime
    based on the location of `libdrake.so` which is loaded by the process.

    This macro will fail-fast if there is ODR violation. This happens if this
    macro adds dependendies (`deps` or `srcs`) that are already part of
    libdrake.so or drake_lcmtypes_headers.
    """
    if not native.package_name().startswith("examples"):
        fail("`drake_example_cc_binary()` macro should only be used in examples \
            subdirectory.")
    # This verifies that there is no ODR violation. Targets that are part of
    # libdrake should not be included a second time. Only targets that are in
    # //examples (historically //drake/examples) or in the workspace can be
    # added as dependencies. By extension, this makes sure that
    # //tools/install/libdrake:drake_shared_library is not added as a
    # dependency a second time.
    for dep in deps:
        if not (dep.startswith('@') or
                dep.startswith(':') or
                dep.startswith('//examples') or
                dep.startswith('//drake/examples')):
            fail("Dependency used in `drake_example_cc_binary()` macro should\
                not already be part of libdrake.so: %s" % dep)
    # This makes sure that //tools/install/libdrake:libdrake.so and
    # //lcmtypes:drake_lcmtypes_headers are not added to srcs a second time.
    if ("//tools/install/libdrake:libdrake.so" in srcs or
            "//lcmtypes:drake_lcmtypes_headers"in srcs):
        fail("//tools/install/libdrake:libdrake.so and \
            //lcmtypes:drake_lcmtypes_headers are already included in \
            `drake_example_cc_binary()` macro")
    drake_cc_binary(
        srcs = srcs,  # + ["//tools/install/libdrake:libdrake.so"],
        deps = deps + [
            "//lcmtypes:drake_lcmtypes_headers",
            "//tools/install/libdrake:drake_shared_library",
        ],
        **kwargs)

def cc_solib_library(
        name,
        solib_name = None,
        hdrs = None,
        srcs = None,
        linkstatic = 0,
        linkshared = 1,
        linkopts = [],
        deps = None,
        # cc_library flags
        include_prefix = None,
        strip_include_prefix = None,
        includes = None,
        # Other flags.
        **kwargs):
    """
    Declares shared library with transitive headers.
    """
    if linkshared != 1:
        fail("`cc_solib_library` only be used with `linkshaed = 1`.")
    if solib_name == None:
        solib_name = "lib{}.so".format(name)
    hdrlib, solib = cc_solib_library_artifacts(name)

    # Headers and upstream dependencies (for transitive consumption).
    native.cc_library(
        name = hdrlib,
        hdrs = hdrs,
        deps = deps,
        include_prefix = include_prefix,
        strip_include_prefix = strip_include_prefix,
        includes = includes,
        **kwargs)
    # Shared library artifact.
    native.cc_binary(
        name = solib_name,
        srcs = srcs,
        linkshared = 1,
        linkstatic = linkstatic,
        linkopts = linkopts,
        deps = [hdrlib],
        **kwargs)
    # Alias library naming for consistent consumption.
    native.alias(
        name = solib,
        actual = solib_name,
    )
    # Development glue.
    native.cc_library(
        name = name,
        srcs = [solib],
        deps = [hdrlib],
        linkstatic = 1,
        **kwargs)
    return (hdrlib, solib)

def cc_solib_library_artifacts(name, private=False):
    """Get targets that should be installed. """
    if not private:
        return [name + ".headers", name + ".solib"]
    else:
        return [name + ".solib"]
