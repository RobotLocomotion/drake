# -*- python -*-

load("@drake//tools/skylark:drake_py.bzl", "py_test_isolated")

# From https://bazel.build/versions/master/docs/be/c-cpp.html#cc_library.srcs
_SOURCE_EXTENSIONS = [source_ext for source_ext in """
.c
.cc
.cpp
.cxx
.c++.C
.h
.hh
.hpp
.hxx
.inc
""".split("\n") if len(source_ext)]

_IGNORE_EXTENSIONS = []

# The cpplint.py command-line argument so it doesn't skip our files!
_EXTENSIONS_ARGS = ["--extensions=" + ",".join(
    [ext[1:] for ext in _SOURCE_EXTENSIONS],
)]

def _extract_labels(srcs):
    """Convert a srcs= or hdrs= value to its set of labels."""

    # Tuples are already labels.
    if type(srcs) == type(()):
        return list(srcs)

    # The select() syntax returns an object we (apparently) can't inspect.
    # TODO(jwnimmer-tri) Figure out how to cpplint these files.  For now,
    # folks will have to pass extra_srcs when calling cpplint() macro.
    return []

def _is_source_label(label):
    for extension in _IGNORE_EXTENSIONS:
        if label.endswith(extension):
            return False
    for extension in _SOURCE_EXTENSIONS:
        if label.endswith(extension):
            return True
    return False

def _add_linter_rules(
        source_labels,
        source_filenames,
        name,
        data = None,
        enable_clang_format_lint = False):
    # Common attributes for all of our py_test invocations.
    data = (data or [])
    size = "small"

    # For cpplint, require a top-level config file.  By default, also apply
    # configs from the current directory and test sub-directory when present.
    # Note that this purposefully uses the _invoking workspace's_ top-level
    # config file (`//:CPPLINT.cfg`), not Drake's `@drake//:CPPLINT.cfg`.
    # (Projects that want their own config can place a CPPLINT.cfg in their
    # root package.  Projects that want to use exactly the Drake defaults can
    # alias Drake's config file into their top-level BUILD.bazel file.)
    cpplint_data = list(data)
    cpplint_cfgs = ["//:CPPLINT.cfg"]
    for x in native.glob(["CPPLINT.cfg", "test/CPPLINT.cfg"]):
        cpplint_cfgs.append("//" + native.package_name() + ":" + x)
    for item in cpplint_cfgs:
        if item not in cpplint_data:
            cpplint_data.append(item)

    # Google cpplint.
    py_test_isolated(
        name = name + "_cpplint",
        srcs = ["@styleguide//:cpplint"],
        data = cpplint_data + source_labels,
        args = _EXTENSIONS_ARGS + source_filenames,
        main = "@styleguide//:cpplint/cpplint.py",
        size = size,
        tags = ["cpplint", "lint"],
    )

    # Additional Drake lint.
    py_test_isolated(
        name = name + "_drakelint",
        srcs = ["@drake//tools/lint:drakelint"],
        data = data + source_labels,
        args = source_filenames,
        main = "@drake//tools/lint:drakelint.py",
        size = size,
        tags = ["drakelint", "lint"],
    )

    # Possibly clang-format idempotence.
    if enable_clang_format_lint:
        py_test_isolated(
            name = name + "_clang_format_lint",
            srcs = ["@drake//tools/lint:clang_format_lint"],
            data = data + source_labels,
            args = source_filenames,
            main = "@drake//tools/lint:clang_format_lint.py",
            size = size,
            tags = ["clang_format_lint", "lint"],
        )

def cpplint(
        existing_rules = None,
        data = None,
        extra_srcs = None,
        enable_clang_format_lint = False):
    """For every rule in the BUILD file so far, adds a test rule that runs
    cpplint over the C++ sources listed in that rule.  Thus, BUILD file authors
    should call this function at the *end* of every C++-related BUILD file.

    By default, only the CPPLINT.cfg from the project root and the current
    directory are used.  Additional configs can be passed in as data labels.

    Sources that are not discoverable through the "sources so far" heuristic
    can be passed in as extra_srcs=[].

    The existing_rules may supply the native.existing_result().values(), in
    case it has already been computed.  When not supplied, the value will be
    internally (re-)computed.

    """
    if existing_rules == None:
        existing_rules = native.existing_rules().values()

    for rule in existing_rules:
        if "nolint" in rule.get("tags"):
            # Disable linting when requested (e.g., for generated code).
            continue
        use_clang_lint = enable_clang_format_lint and (
            "nolint_clang_format" not in rule.get("tags")
        )

        # Extract the list of C++ source code labels and convert to filenames.
        candidate_labels = (
            _extract_labels(rule.get("srcs", ())) +
            _extract_labels(rule.get("hdrs", ()))
        )
        source_labels = [
            label
            for label in candidate_labels
            if _is_source_label(label)
        ]
        source_filenames = ["$(location %s)" % x for x in source_labels]

        # Run the cpplint checker as a unit test.
        if len(source_filenames) > 0:
            _add_linter_rules(
                source_labels,
                source_filenames,
                name = rule["name"],
                data = data,
                enable_clang_format_lint = use_clang_lint,
            )

    # Lint all of the extra_srcs separately in a single rule.
    if extra_srcs:
        source_labels = extra_srcs
        source_filenames = ["$(location %s)" % x for x in source_labels]
        _add_linter_rules(
            source_labels,
            source_filenames,
            name = "extra_srcs_cpplint",
            data = data,
            enable_clang_format_lint = enable_clang_format_lint,
        )
