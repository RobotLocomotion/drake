load("//tools/skylark:drake_py.bzl", "py_test_isolated")

# From https://bazel.build/reference/be/c-cpp#cc_library.srcs.
# Keep this list in sync with clang_format_lint.py.
_SOURCE_EXTENSIONS = [source_ext for source_ext in """
.c
.cc
.cpp
.cxx
.c++
.C
.h
.hh
.hpp
.hxx
.inc
.inl
.H
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
        data = None):
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
    for x in native.glob(
        ["CPPLINT.cfg", "test/CPPLINT.cfg"],
        allow_empty = True,
    ):
        cpplint_cfgs.append("//" + native.package_name() + ":" + x)
    for item in cpplint_cfgs:
        if item not in cpplint_data:
            cpplint_data.append(item)

    # Google cpplint.
    py_test_isolated(
        name = name + "_cpplint",
        srcs = ["@cpplint_internal//:cpplint"],
        data = cpplint_data + source_labels,
        args = _EXTENSIONS_ARGS + source_filenames,
        main = "@cpplint_internal//:cpplint.py",
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

    # Idempotence during clang-format.
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
        extra_srcs = None):
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

    # Collect the list of files to lint, grouped by target name.
    name_to_labels = {}
    for rule in existing_rules:
        if "nolint" in rule.get("tags"):
            # Disable linting when requested (e.g., for generated code).
            continue

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
        if len(source_labels) == 0:
            continue

        # Undo the effect of _maybe_add_pruned_private_hdrs_dep, which computes
        # its `name = "_" + base_name + "_private_headers_cc_impl"`.
        rule_name = rule["name"]
        private_suffix = "_private_headers_cc_impl"
        if rule_name.endswith(private_suffix):
            rule_name = rule_name[1:-len(private_suffix)]

        # Note the files to be linted.
        name_to_labels.setdefault(rule_name, [])
        name_to_labels[rule_name].extend(source_labels)

    # Run the cpplint checker as unit tests.
    for rule_name, source_labels in name_to_labels.items():
        source_filenames = ["$(location %s)" % x for x in source_labels]
        _add_linter_rules(
            source_labels,
            source_filenames,
            name = rule_name,
            data = data,
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
        )

def cpplint_extra(
        name,
        *,
        srcs):
    """Adds a test rule that runs cpplint over the given C++ code.
    The srcs can be any kind of source code (i.e., header and/or cc files).
    """
    _add_linter_rules(
        name = name,
        source_labels = srcs,
        source_filenames = ["$(location %s)" % x for x in srcs],
    )
