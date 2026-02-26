load(
    "//tools/skylark:cc.bzl",
    "CcInfo",
    "cc_binary",
    "cc_common",
    "cc_library",
    "cc_test",
)
load(
    "//tools/skylark:kwargs.bzl",
    "incorporate_allow_network",
    "incorporate_display",
    "incorporate_num_threads",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

# The CXX_FLAGS will be enabled for all C++ rules in the project
# building with any compiler.
CXX_FLAGS = [
    "-Werror=all",
    "-Werror=attributes",
    "-Werror=cpp",
    "-Werror=deprecated",
    "-Werror=deprecated-declarations",
    "-Werror=ignored-qualifiers",
    "-Werror=missing-declarations",
    "-Werror=overloaded-virtual",
    "-Werror=shadow",
    "-Werror=unused-result",
    # We eschew Eigen::IO in lieu of drake::fmt_eigen.
    # See drake/common/fmt_eigen.h for details.
    "-DEIGEN_NO_IO=1",
]

# The CLANG_FLAGS will be enabled for all C++ rules in the project when
# building with clang (excluding the Apple LLVM compiler see APPLECLANG_FLAGS
# below).
CLANG_FLAGS = CXX_FLAGS + [
    "-Werror=absolute-value",
    "-Werror=c99-designator",
    "-Werror=inconsistent-missing-override",
    "-Werror=final-dtor-non-final-class",
    "-Werror=literal-conversion",
    "-Werror=non-virtual-dtor",
    "-Werror=range-loop-analysis",
    "-Werror=return-stack-address",
    "-Werror=sign-compare",
    "-Werror=unqualified-std-cast-call",
]

# The APPLECLANG_FLAGS will be enabled for all C++ rules in the project when
# building with the Apple LLVM compiler.
APPLECLANG_FLAGS = CLANG_FLAGS

# The GCC_FLAGS will be enabled for all C++ rules in the project when
# building with gcc.
GCC_FLAGS = CXX_FLAGS + [
    "-Werror=extra",
    "-Werror=logical-op",
    "-Werror=non-virtual-dtor",
    "-Werror=return-local-addr",
    "-Werror=unused-but-set-parameter",
    # This was turned on via -Wextra, but is too strict to have as an error.
    "-Wno-missing-field-initializers",
]

# The CC_TEST_FLAGS will be enabled for all cc_test rules in the project.
CC_TEST_FLAGS = [
    "-Wno-sign-compare",
]

# The GCC_CC_TEST_FLAGS will be enabled for all cc_test rules in the project
# when building with gcc.
GCC_CC_TEST_FLAGS = [
    "-Wno-missing-declarations",
    "-Wno-unused-parameter",
]

# The GCC_13_OR_NEWER_FLAGS will be enabled for all C++ rules in the project
# when building with gcc 13 through the newest version in the official support
# matrix. See GCC_VERSION_SPECIFIC_FLAGS below for details.
GCC_13_OR_NEWER_FLAGS = [
    "-Werror=pessimizing-move",
    "-Werror=uninitialized",
    # This falsely dings code that returns const references, e.g., our
    # MbP style for "add element" or "find by name" member functions.
    "-Wno-dangling-reference",
    # This falsely dings code inside Eigen.
    "-Wno-maybe-uninitialized",
    # This falsely dings code inside libstdc++.
    "-Wno-stringop-overflow",
    # These two falsely ding initializing an Eigen::Vector1d or Matrix1d.
    # Eigen uses 16-byte alignment, which these flags doesn't account for.
    "-Wno-array-bounds",
    "-Wno-stringop-overread",
]

# The GCC_VERSION_SPECIFIC_FLAGS will be enabled for all C++ rules in the
# project when building with gcc of the specified major version, but only if
# the --@drake//tools/cc_toolchain:compiler_major=NN flag has been set on the
# command line or in an rcfile. (It typically will be except when Drake is used
# as a Bazel external.)
GCC_VERSION_SPECIFIC_FLAGS = {
    13: GCC_13_OR_NEWER_FLAGS,
    14: GCC_13_OR_NEWER_FLAGS,
    15: GCC_13_OR_NEWER_FLAGS,
}

def _defang(flags):
    """Given a list of copts, demote all -Werror into just plain -W."""
    return [
        x.replace("-Werror=", "-W")
        for x in flags
    ]

# The BASE_COPTS are used for all drake_cc_{binary,library,test} rules.
BASE_COPTS = select({
    "//tools/cc_toolchain:apple_clang_with_errors": APPLECLANG_FLAGS,
    "//tools/cc_toolchain:apple_clang_with_warnings": _defang(APPLECLANG_FLAGS),
    "//tools/cc_toolchain:gcc_with_errors": GCC_FLAGS,
    "//tools/cc_toolchain:gcc_with_warnings": _defang(GCC_FLAGS),
    "//tools/cc_toolchain:linux_clang_with_errors": CLANG_FLAGS,
    "//tools/cc_toolchain:linux_clang_with_warnings": _defang(CLANG_FLAGS),
    "//conditions:default": _defang(CXX_FLAGS),
}) + select(dict([
    ("//tools/cc_toolchain:gcc_{}_with_errors".format(major_ver), flags)
    for major_ver, flags in GCC_VERSION_SPECIFIC_FLAGS.items()
] + [
    ("//tools/cc_toolchain:gcc_{}_with_warnings".format(major_ver), _defang(flags))  # noqa
    for major_ver, flags in GCC_VERSION_SPECIFIC_FLAGS.items()
] + [
    ("//conditions:default", []),
]))

def _platform_copts(rule_copts, rule_gcc_copts, rule_clang_copts, cc_test = 0):
    """Returns the concatenation of Drake's platform-specific BASE_COPTS,
    plus the passed-in rule_copts, plus the passed-in rule_{cc}_copts iff
    building with the specified compiler).

    When cc_test=1, the GCC_CC_TEST_FLAGS will also be added. It should only be
    used from cc_test rules or rules that boil down to cc_test rules.
    """
    if not any([rule_copts, rule_gcc_copts, rule_clang_copts, cc_test]):
        # In the case of no special arguments at all, we can save Bazel the
        # hassle of concatenating a bunch of empty stuff.
        return BASE_COPTS
    test_gcc_copts = (CC_TEST_FLAGS + GCC_CC_TEST_FLAGS) if cc_test else []
    test_clang_copts = CC_TEST_FLAGS if cc_test else []
    return BASE_COPTS + rule_copts + select({
        "@rules_cc//cc/compiler:gcc": rule_gcc_copts + test_gcc_copts,
        "@rules_cc//cc/compiler:clang": rule_clang_copts + test_clang_copts,
        "//conditions:default": [],
    })

# The BASE_LINKOPTS are used for all drake_cc_{binary,library,test} rules.
BASE_LINKOPTS = select({
    "@drake//tools/cc_toolchain:use_mold_linker": [
        "-fuse-ld=mold",
        "-Wl,--compress-debug-sections=zlib",
        "-Wl,--thread-count=2",
    ],
    "//conditions:default": [],
})

def _check_library_deps_blacklist(name, deps):
    """Report an error if a library should not use something from deps."""
    if not deps:
        return
    if type(deps) != "list":
        # We can't handle select() yet.
        # TODO(jwnimmer-tri) We should handle select.
        return
    for dep in deps:
        if name == "drake_cc_googletest_main":
            # This library-with-main is a special case.
            continue
        if dep.endswith(":add_text_logging_gflags"):
            fail("The cc_library '" + name + "' must not depend on " +
                 "//common:add_text_logging_gflags; only cc_binary targets " +
                 "are allowed to have gflags")
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
    `dep` for a cc_library, such as would be found in the `deps = []` of
    some cc_library, returns the corresponding label for the matching DrakeCc
    provider associated with that library.  The returned label is appropriate
    to use in the deps of a `drake_installed_headers()` rule.

    Once our rules are better able to call native rules like native cc_binary,
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
    return result

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
        for x in deps
        if (
            not x.startswith("@") and
            not x.startswith("//drake/lcmtypes:") and
            not x == "//:drake_shared_library" and
            not x.startswith("//third_party")
        )
    ]

# A provider to collect Drake metadata about C++ rules.  For background, see
# https://docs.bazel.build/versions/master/skylark/rules.html#providers.
DrakeCc = provider()

def _drake_installed_headers_impl(ctx):
    hdrs = list(ctx.files.hdrs)
    for x in ctx.files.hdrs_exclude:
        hdrs.remove(x)
    transitive_hdrs = depset(hdrs, transitive = [
        dep[DrakeCc].transitive_hdrs
        for dep in ctx.attr.deps
    ])
    return [
        DrakeCc(
            transitive_hdrs = transitive_hdrs,
        ),
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
        "deps": attr.label_list(
            mandatory = True,
            providers = [DrakeCc],
        ),
    },
    implementation = _drake_installed_headers_impl,
)

def _path_startswith_match(path, only_startswith, never_startswith):
    # Ignore some leading path elements.  These will happen if Drake is
    # consumed as an external.
    strip = "../drake+/"
    if path.startswith(strip):
        path = path[len(strip):]

    # Returns true iff `path` is consistent with the given `only...` and
    # `never...` prefixes.  Omitting either or both of the `...startswith`
    # arguments is treated as a pass (true) by default.
    if only_startswith:
        if not path.startswith(only_startswith):
            return False
    for prefix in never_startswith:
        if path.startswith(prefix):
            return False
    return True

def _gather_transitive_hdrs_impl(ctx):
    # Transitively list all headers.
    all_hdrs = depset([], transitive = [
        dep[DrakeCc].transitive_hdrs
        for dep in ctx.attr.deps
    ])

    # Filter in/out items matching a prefix.
    result = depset([
        x
        for x in all_hdrs.to_list()
        if _path_startswith_match(
            x.short_path,
            ctx.attr.only_startswith,
            ctx.attr.never_startswith,
        )
    ])

    return [
        DefaultInfo(
            files = result,
            runfiles = ctx.runfiles(transitive_files = result),
        ),
    ]

_gather_transitive_hdrs = rule(
    attrs = {
        "deps": attr.label_list(
            allow_files = False,
            providers = [DrakeCc],
        ),
        "only_startswith": attr.string(),
        "never_startswith": attr.string_list(),
    },
    implementation = _gather_transitive_hdrs_impl,
)

def drake_transitive_installed_hdrs_filegroup(
        name,
        deps = [],
        only_startswith = None,
        never_startswith = [],
        **kwargs):
    """Declare a filegroup that contains the transtive installed hdrs of the
    targets named by `deps`.
    """
    _gather_transitive_hdrs(
        name = name + "_gather",
        deps = [installed_headers_for_dep(x) for x in deps],
        visibility = [],
        only_startswith = only_startswith,
        never_startswith = never_startswith,
    )
    native.filegroup(
        name = name,
        srcs = [":" + name + "_gather"],
        **kwargs
    )

def _cc_linkonly_library_impl(ctx):
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                collect_data = True,
                collect_default = True,
            ),
        ),
        CcInfo(
            compilation_context = None,
            linking_context = deps_cc_infos.linking_context,
        ),
    ]

cc_linkonly_library = rule(
    implementation = _cc_linkonly_library_impl,
    doc = """
Links the given dependencies but discards the entire compilation context,
i.e., include paths and preprocessor definitions.
""",
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
    },
    fragments = ["cpp"],
)

def _cc_nolink_library_impl(ctx):
    deps_cc_infos = cc_common.merge_cc_infos(
        cc_infos = [dep[CcInfo] for dep in ctx.attr.deps],
    )
    return [
        DefaultInfo(
            runfiles = ctx.runfiles(
                collect_data = True,
                collect_default = True,
            ),
        ),
        CcInfo(
            compilation_context = deps_cc_infos.compilation_context,
            linking_context = None,
        ),
    ]

cc_nolink_library = rule(
    implementation = _cc_nolink_library_impl,
    doc = """
Provides access to the header files and preprocessor definitions from the given
dependencies but discards all linked libraries and linker options.
""",
    attrs = {
        "deps": attr.label_list(providers = [CcInfo]),
    },
    fragments = ["cpp"],
)

def _raw_drake_cc_library(
        name,
        srcs = None,  # Cannot list any headers here.
        hdrs = None,
        strip_include_prefix = None,
        include_prefix = None,
        copts = None,
        defines = None,
        data = None,
        deps = None,
        implementation_deps = None,
        linkstatic = None,
        linkopts = None,
        alwayslink = None,
        tags = None,
        testonly = None,
        visibility = None,
        compile_once_per_scalar = False,
        declare_installed_headers = None,
        install_hdrs_exclude = None,
        deprecation = None):
    """Creates a rule to declare a C++ library.  Uses Drake's include_prefix
    and checks the deps blacklist.  If declare_installed_headers is true, also
    adds a drake_installed_headers() target.  (This should be set if and only
    if the caller is drake_cc_library.)
    """
    _check_library_deps_blacklist(name, deps)
    _check_library_deps_blacklist(name, implementation_deps)
    _, private_hdrs = _prune_private_hdrs(srcs)
    if private_hdrs:
        fail("private_hdrs = " + private_hdrs)

    # Require include paths like "drake/foo/bar.h", not "foo/bar.h".
    if strip_include_prefix == None:
        strip_include_prefix = "/"
    if include_prefix == None:
        include_prefix = "drake"

    # Add the installed header tracking, unless we've opted-out.
    if declare_installed_headers:
        drake_installed_headers(
            name = name + ".installed_headers",
            hdrs = hdrs,
            hdrs_exclude = install_hdrs_exclude,
            deps = installed_headers_for_drake_deps(deps),
            tags = ["nolint"],
            visibility = ["//visibility:public"],
        )

    # When compiling using once_per_scalar, we need to replace the srcs with
    # small stub files that set the scalar type first. (Note that this block of
    # code will crash if srcs uses a `select`; that usage is not supported.)
    textual_hdrs = None
    if compile_once_per_scalar:
        new_srcs = []
        for src in srcs:
            for i in range(3):
                stub_name = "{}_{}".format(i, src)
                generate_file(
                    name = stub_name,
                    content = (
                        ("#define DRAKE_ONCE_PER_SCALAR_PHASE {}\n" +
                         "#include \"{}/{}\"\n").format(
                            i,
                            native.package_name(),
                            src,
                        )
                    ),
                    visibility = ["//visibility:private"],
                )
                new_srcs.append(stub_name)

        # Don't lint the stubs; instead, use a dummy rule to do the linting.
        # We use hdrs for everything so that the linter can see it but nothing
        # will actually be compiled.
        cc_library(
            name = "{}_for_linting".format(name),
            hdrs = (hdrs or []) + (srcs or []),
            tags = ["manual", "exclude_from_package"],
            visibility = ["//visibility:private"],
        )
        tags = (tags or []) + ["nolint"]

        # The old srcs convert to textual_hdrs; the stubs are the new srcs.
        textual_hdrs = srcs
        srcs = new_srcs

    # Finally, do the actual compilation.
    cc_library(
        name = name,
        srcs = srcs,
        hdrs = hdrs,
        textual_hdrs = textual_hdrs,
        strip_include_prefix = strip_include_prefix,
        include_prefix = include_prefix,
        copts = copts,
        defines = defines,
        data = data,
        deps = deps,
        implementation_deps = implementation_deps,
        linkstatic = linkstatic,
        linkopts = linkopts,
        alwayslink = alwayslink,
        tags = tags,
        testonly = testonly,
        visibility = visibility,
        deprecation = deprecation,
    )

def _maybe_add_pruned_private_hdrs_dep(
        base_name,
        srcs,
        deps,
        **kwargs):
    """Given some srcs, prunes any header files into a separate cc_library, and
    appends that new library to deps, returning new_srcs (sans headers) and
    add_deps.  The separate cc_library is private with linkstatic = 1.

    We use this helper in all drake_cc_{library,binary,test) because when we
    want to fiddle with include paths, we *must* have all header files listed
    as hdrs; the include_prefix does not apply to srcs.
    """
    new_srcs, private_hdrs = _prune_private_hdrs(srcs)
    if private_hdrs:
        name = "_" + base_name + "_private_headers_cc_impl"
        kwargs.pop("env", "")
        kwargs.pop("linkshared", "")
        kwargs.pop("linkstatic", "")
        kwargs.pop("visibility", "")
        _raw_drake_cc_library(
            name = name,
            hdrs = private_hdrs,
            srcs = [],
            deps = deps,
            linkstatic = 1,
            visibility = ["//visibility:private"],
            **kwargs
        )
        add_deps = [":" + name]
    else:
        add_deps = []
    return new_srcs, add_deps

def drake_cc_library(
        name,
        hdrs = [],
        srcs = [],
        deps = [],
        implementation_deps = None,
        copts = [],
        clang_copts = [],
        gcc_copts = [],
        linkopts = [],
        linkstatic = 1,
        internal = False,
        compile_once_per_scalar = False,
        declare_installed_headers = 1,
        install_hdrs_exclude = [],
        **kwargs):
    """Creates a rule to declare a C++ library.

    By default, we produce only static libraries, to reduce compilation time
    on all platforms, and to avoid mysterious dyld errors on OS X. This default
    could be revisited if binary size becomes a concern.

    The dependencies of a cc_library can be classified as "interface deps" or
    "implementation deps". Files that are #include'd by headers are "interface
    deps"; files that are #include'd by srcs are "implementation deps".

    Dependencies listed as "implementation deps" will be linked into your
    program, but their header files will only be available to the srcs files
    here. Downstream users of this library will not have access to the
    implementation deps's header files, nor will any preprocessor definitions
    for the implementation deps propagate past this firewall.

    When declaring a build rule, the spelling for "interface deps" is `deps =`
    and for "implemenetation deps" is `implementation_deps =`.

    The dependencies of a drake_cc_library must be another drake_cc_library, or
    else be named like "@something//etc..." (i.e., come from the workspace, not
    part of Drake).  In other words, all of Drake's C++ libraries must be
    declared using the drake_cc_library macro.

    Setting `internal = True` is convenient sugar to flag code that is never
    used by Drake's installed headers. This is especially helpful when the
    header_lint tool is complaining about third-party dependency pollution.
    Flagging a library as `internal = True` means that the library is internal
    from the point of view of the build system, which is an even stronger
    promise that merely placing code inside `namespace internal {}`.
    Code that is build-system internal should always be namespace-internal,
    but not all namespace-internal code is build-system internal. For example,
    drake/common/diagnostic_policy.h is namespace-internal, but cannot be
    build-internal because other Drake headers (multibody/parsing/parser.h)
    refer to the diagnostic policy header, for use by their private fields.
    Libraries marked with `internal = True` should generally be listed only as
    deps of _other_ libraries marked as internal, or as "implementation deps"
    (see paragraphs above) of non-internal libraries.

    Setting `compile_once_per_scalar = True` shards the library rule to build
    each file three times (once per scalar, as separate translation units),
    instead of compiling all scalars within the same translation unit. This
    reduces build latency for especially large source files. Code in cc files
    that is not templated (and therefore should be not be compiled three times)
    should be surrounded with `#if DRAKE_ONCE_PER_SCALAR_PHASE == 0`.
    """
    new_copts = _platform_copts(copts, gcc_copts, clang_copts)
    new_linkopts = BASE_LINKOPTS + linkopts
    new_tags = kwargs.pop("tags", None) or []
    if internal:
        if install_hdrs_exclude != []:
            fail("When using internal = True, hdrs are automatically excluded")
        if kwargs.get("testonly", False):
            fail("Using internal = True is already implied under testonly = 1")
        if len(kwargs.get("visibility") or []) == 0:
            fail("When using internal = True, you must set visibility. " +
                 "In most cases, visibility = [\"//visibility:private\"] " +
                 "or visibility = [\"//:__subpackages__\"] are suitable.")
        for item in kwargs["visibility"]:
            if item == "//visibility:public":
                fail("When using internal = True, visibility can't be public")
        install_hdrs_exclude = hdrs
        new_tags = new_tags + [
            "exclude_from_libdrake",
            "exclude_from_package",
        ]

    # We install private_hdrs by default, because Bazel's visibility denotes
    # whether headers can be *directly* included when using cc_library; it does
    # not precisely relate to which headers should appear in the install tree.
    # For example, common/symbolic.h is the only public-visibility header for
    # its cc_library, but we also need to install all of its child headers that
    # it includes, such as common/symbolic_expression.h.
    new_srcs, add_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = deps,
        copts = new_copts,
        linkopts = new_linkopts,
        declare_installed_headers = declare_installed_headers,
        tags = new_tags,
        **kwargs
    )
    _raw_drake_cc_library(
        name = name,
        hdrs = hdrs,
        srcs = new_srcs,
        deps = deps + add_deps,
        implementation_deps = implementation_deps,
        copts = new_copts,
        linkopts = new_linkopts,
        linkstatic = linkstatic,
        declare_installed_headers = declare_installed_headers,
        install_hdrs_exclude = install_hdrs_exclude,
        compile_once_per_scalar = compile_once_per_scalar,
        tags = new_tags,
        **kwargs
    )

def _check_package_library_name(name):
    # Assert that :name is the default library for native.package_name().
    expected_name = native.package_name().split("/")[-1]
    if name != expected_name:
        fail(("The drake_cc_package_library(name = \"{}\", ...) " +
              "should be named \"{}\"").format(name, expected_name))

def drake_cc_package_library(
        name,
        deps = [],
        testonly = 0,
        visibility = None):
    """Creates a rule to declare a C++ "package" library -- a library whose
    name matches the current Bazel package name (i.e., directory name) and
    whose dependencies are (usually) all of the other drake_cc_library targets
    in the current package.  In short, e.g., creates a library named
    //foo/bar:bar that conveniently provides all of the C++ code from the
    //foo/bar package in one place.

    Using this macro documents the intent that the library is a summation of
    everything in the current package and enables Drake's linter rules to
    confirm that all of the drake_cc_library targets have been listed as deps.

    Within Drake, by convention, every package (i.e., directory) that has any
    C++ code should call this macro to create a library for its package.

    The name must be the same as the final element of the current package.
    This rule does not accept srcs, hdrs, etc. -- only deps.
    The testonly argument has the same meaning as the native cc_library.

    The visibility must be explicitly provided, not relying on the BUILD file
    default.  Setting to "//visibility:public" is strongly recommended.
    """
    _check_package_library_name(name)
    if not visibility:
        fail(("//{}:{} must provide a visibility setting; " +
              "add this line to the BUILD.bazel file:\n" +
              "        visibility = \"//visibility:public\",").format(
            native.package_name(),
            name,
        ))
    drake_cc_library(
        name = name,
        testonly = testonly,
        tags = ["drake_cc_package_library"],
        visibility = visibility,
        deps = deps,
    )

def drake_cc_binary(
        name,
        srcs = [],
        data = [],
        deps = [],
        copts = [],
        linkopts = [],
        gcc_copts = [],
        clang_copts = [],
        linkshared = 0,
        linkstatic = 1,
        testonly = 0,
        add_test_rule = 0,
        test_rule_args = [],
        test_rule_data = [],
        test_rule_tags = None,
        test_rule_size = None,
        test_rule_timeout = None,
        test_rule_flaky = 0,
        **kwargs):
    """Creates a rule to declare a C++ binary.

    By default, we prefer to link static libraries whenever they are available.
    This default could be revisited if binary size becomes a concern.

    Note that drake_cc_binary does not draw any distinction between "interface
    deps" vs "implementation deps". There is no "interface", because this is a
    binary, not a library.

    If you wish to create a smoke-test demonstrating that your binary runs
    without crashing, supply add_test_rule=1. Note that if you wish to do
    this, you should consider suppressing that urge, and instead writing real
    tests. The smoke-test will be named <name>_test. You may override cc_test
    defaults using test_rule_args=["-f", "--bar=42"] or test_rule_size="baz".
    """
    new_copts = _platform_copts(copts, gcc_copts, clang_copts)
    new_linkopts = BASE_LINKOPTS + linkopts
    new_srcs, add_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = deps,
        copts = new_copts,
        linkopts = new_linkopts,
        testonly = testonly,
        **kwargs
    )

    cc_binary(
        name = name,
        srcs = new_srcs,
        data = data,
        deps = deps + add_deps,
        copts = new_copts,
        testonly = testonly,
        linkshared = linkshared,
        linkstatic = linkstatic,
        linkopts = new_linkopts,
        features = [
            # We should deduplicate symbols while linking (for a ~6% reduction
            # in disk use), to conserve space in CI; see #18545 for details.
            "-no_deduplicate",
        ],
        **kwargs
    )

    if "@googletest//:gtest_main" in deps:
        fail("Use drake_cc_googletest to declare %s as a test" % name)

    if add_test_rule:
        drake_cc_test(
            name = name + "_test",
            srcs = new_srcs,
            data = data + test_rule_data,
            deps = deps + add_deps,
            copts = copts,
            linkopts = new_linkopts,
            gcc_copts = gcc_copts,
            size = test_rule_size,
            timeout = test_rule_timeout,
            flaky = test_rule_flaky,
            linkstatic = linkstatic,
            args = test_rule_args,
            tags = (test_rule_tags or []) + ["nolint", "no_kcov"],
            **kwargs
        )

def drake_cc_test(
        name,
        size = None,
        srcs = [],
        args = [],
        deps = [],
        copts = [],
        gcc_copts = [],
        clang_copts = [],
        linkopts = [],
        allow_network = None,
        display = False,
        num_threads = None,
        **kwargs):
    """Creates a rule to declare a C++ unit test.  Note that for almost all
    cases, drake_cc_googletest should be used, instead of this rule.

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.
    Unconditionally forces testonly=1.

    @param allow_network (optional, default is ["meshcat"])
        See drake/tools/skylark/README.md for details.

    @param display (optional, default is False)
        See drake/tools/skylark/README.md for details.

    @param num_threads (optional, default is 1)
        See drake/tools/skylark/README.md for details.
    """
    if size == None:
        size = "small"
    if not srcs:
        srcs = ["test/%s.cc" % name]
    kwargs["testonly"] = 1
    kwargs = incorporate_allow_network(kwargs, allow_network = allow_network)
    kwargs = incorporate_display(kwargs, display = display)
    kwargs = incorporate_num_threads(kwargs, num_threads = num_threads)
    new_copts = _platform_copts(copts, gcc_copts, clang_copts, cc_test = 1)
    new_linkopts = BASE_LINKOPTS + linkopts
    new_srcs, add_deps = _maybe_add_pruned_private_hdrs_dep(
        base_name = name,
        srcs = srcs,
        deps = deps,
        copts = new_copts,
        linkopts = new_linkopts,
        **kwargs
    )
    cc_test(
        name = name,
        size = size,
        srcs = new_srcs,
        args = args,
        deps = deps + add_deps,
        copts = new_copts,
        linkopts = new_linkopts,
        features = [
            # We should deduplicate symbols while linking (for a ~6% reduction
            # in disk use), to conserve space in CI; see #18545 for details.
            "-no_deduplicate",
        ],
        **kwargs
    )

def drake_cc_googletest(
        name,
        args = [],
        tags = [],
        deps = [],
        disable_in_compilation_mode_dbg = False,
        use_default_main = True,
        **kwargs):
    """Creates a rule to declare a C++ unit test using googletest.

    By default, sets size="small" because that indicates a unit test.
    By default, sets name="test/${name}.cc" per Drake's filename convention.
    By default, sets use_default_main=True to use a default main() function.
    Otherwise, it will depend on @googletest//:gtest.

    If disable_in_compilation_mode_dbg is True, then in debug-mode builds all
    test cases will be suppressed, so the test will trivially pass. This option
    should be used only rarely, and the reason should always be documented.
    """
    if use_default_main:
        deps = deps + [
            "//common/test_utilities:drake_cc_googletest_main",
        ]
    else:
        deps = deps + ["@googletest//:gtest"]
    new_args = args
    new_tags = tags
    if disable_in_compilation_mode_dbg:
        # If we're in debug compilation mode, then skip all test cases so that
        # the test will trivially pass.
        new_args = args + select({
            "//tools/cc_toolchain:debug": ["--gtest_filter=-*"],
            "//conditions:default": [],
        })

        # Skip this test when run under various dynamic tools that use
        # debug-like compiler flags.
        new_tags = new_tags + [
            "no_asan",
            "no_kcov",
            "no_lsan",
            "no_memcheck",
            "no_tsan",
            "no_ubsan",
        ]
    else:
        # kcov is only appropriate for small-sized unit tests. If a test needs
        # a shard_count or a special timeout, we assume it is not small.
        if "shard_count" in kwargs or "timeout" in kwargs:
            new_tags = new_tags + ["no_kcov"]

    drake_cc_test(
        name = name,
        args = new_args,
        tags = new_tags,
        deps = deps,
        **kwargs
    )
