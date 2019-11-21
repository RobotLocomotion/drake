# -*- python -*-

load("//tools/skylark:py.bzl", "py_library")
load(
    "@drake//tools/skylark:drake_cc.bzl",
    "drake_cc_library",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

def _combine_relative_labels(arg_list, arg_map):
    # Merge the relative_labels= and relative_labels_map= arguments as seen in
    # the public macros below.  The result is a map where the arg_list items
    # are twinned into matching key:key pairs, union'd with the arg_map.  (Also
    # double-checks that the keys and values are all relative labels.)
    result = dict([(x, x) for x in arg_list])
    result.update(arg_map)
    for x in result.keys() + result.values():
        if not x.startswith(":"):
            fail("Expected relative_label, got " + x)
    return result

def drake_cc_hdrs_forwarding_library(
        name,
        relative_labels = [],
        relative_labels_map = {},
        actual_subdir = "",
        add_deprecation_warning = False,
        tags = [],
        **kwargs):
    """Generates a drake_cc_library filled with generated header files that
    merely include other header files, optionally with a deprecation warning.
    This automates the chore of adding compatibility headers when packages
    move.

    The relative_labels + relative_labels_map.keys() specify the header
    filenames to generate.  For each label name ':foo', this macro generates a
    header named 'foo.h' that includes 'drake/{actual_subdir}/{new_foo}.h'
    where new_foo for relative_labels is the same as foo, and new_foo for
    relative_labels_map is the value for the foo key (without the leading
    colon).

    When add_deprecation_warning is true, the generated header will have a
    #warning directive explaining the new location.
    """

    actual_subdir or fail("Missing required actual_subdir")

    # Generate one header file for each mapped label.
    hdrs = []
    mapping = _combine_relative_labels(relative_labels, relative_labels_map)
    for stub_relative_label, actual_relative_label in mapping.items():
        actual_include = "drake/{}/{}.h".format(
            actual_subdir,
            actual_relative_label[1:],
        )
        warning = "#warning This header is deprecated; use {} instead".format(
            actual_include,
        ) if add_deprecation_warning else ""
        basename = stub_relative_label[1:] + ".h"
        generate_file(
            name = basename,
            content = "\n".join([
                "#pragma once",
                "{warning}",
                "#include \"{actual_include}\"",
            ]).format(
                warning = warning,
                actual_include = actual_include,
            ),
        )
        hdrs.append(basename)

    # Place all of the header files into a library.
    drake_cc_library(
        name = name,
        hdrs = hdrs,
        tags = tags + ["nolint"],
        **kwargs
    )

def drake_cc_library_aliases(
        relative_labels = [],
        relative_labels_map = {},
        actual_subdir = "",
        add_deprecation_warning = False,
        tags = [],
        deps = [],
        **kwargs):
    """Generates aliases for drake_cc_library labels that have moved into a new
    package.  This is superior to native.alias() both for its brevity (it can
    generate many aliases at once), and because the deprecation message
    actually works (see https://github.com/bazelbuild/bazel/issues/5802).

    The relative_labels + relative_labels_map specify the correspondence.  For
    each label name foo in the relative_labels and relative_labels_map.keys(),
    this macro generates an alias to '//{actual_subdir}:{new_foo}' where
    new_foo for relative_labels is the same as foo, and new_foo for
    relative_labels_map is the value for the foo key.

    When add_deprecation_warning is true, the generated label in this package
    will have a deprecation warning explaining the new location.  When using
    this mode, the caller should probably set 'tags = ["manual"]' to avoid
    spurious warnings during 'bazel build //...'.
    """

    actual_subdir or fail("Missing required actual_subdir")

    mapping = _combine_relative_labels(relative_labels, relative_labels_map)
    for stub_relative_label, actual_relative_label in mapping.items():
        actual_full_label = "//" + actual_subdir + actual_relative_label
        if add_deprecation_warning:
            deprecation = ("Use the label " + actual_full_label)
        else:
            deprecation = None
        native.cc_library(
            name = stub_relative_label[1:],
            deps = deps + [actual_full_label],
            tags = tags + ["nolint"],
            deprecation = deprecation,
            **kwargs
        )
        native.alias(
            name = stub_relative_label[1:] + ".installed_headers",
            actual = actual_full_label + ".installed_headers",
        )

_PY_TEMPLATE_NOT_DEPRECATED = r'''"""
Warning:
    This module is an alias that will soon be deprecated.
    Please use ``{module}`` instead.
"""
from {module} import *
'''

_PY_TEMPLATE_DEPRECATED = r'''"""
Warning:
    This module is deprecated and will be removed on or around
    {deprecation_removal_date}. Please use ``{module}`` instead.
"""
from pydrake.common.deprecation import _warn_deprecated
from {module} import *

_warn_deprecated(
    "This module is deprecated and will be removed on or around "
    "{deprecation_removal_date}. Please use '{module}' instead.",
    stacklevel=3)
'''

def _strip_py_suffix(label):
    if not label.endswith("_py"):
        fail("'{}' must end with '_py'".format(label))
    return label[1:-len("_py")]

def drake_py_library_aliases(
        relative_labels = [],
        relative_labels_map = {},
        actual_subdir = "",
        add_deprecation_warning = False,
        deprecation_removal_date = None,
        tags = [],
        **kwargs):
    actual_subdir or fail("Missing required actual_subdir")
    subdir_prefix = "bindings/pydrake/"
    if not actual_subdir.startswith(subdir_prefix):
        fail("'{}' must start with '{}'".format(actual_subdir, subdir_prefix))
    if actual_subdir.endswith("/"):
        fail("'{}' must not end with '/'".format(actual_subdir))
    actual_package = actual_subdir[len("bindings/"):].replace("/", ".")
    mapping = _combine_relative_labels(relative_labels, relative_labels_map)
    for stub_relative_label, actual_relative_label in mapping.items():
        stub_file = _strip_py_suffix(stub_relative_label) + ".py"
        actual_name = _strip_py_suffix(actual_relative_label)
        actual_module = actual_package + "." + actual_name
        if add_deprecation_warning:
            if deprecation_removal_date == None:
                fail("`deprecation_removal_date` must be supplied.")
            generate_file(
                name = stub_file,
                content = _PY_TEMPLATE_DEPRECATED.format(
                    module = actual_module,
                    deprecation_removal_date = deprecation_removal_date,
                ),
            )
        else:
            generate_file(
                name = stub_file,
                content = _PY_TEMPLATE_NOT_DEPRECATED.format(
                    module = actual_module,
                ),
            )
        actual_full_label = "//" + actual_subdir + actual_relative_label
        py_library(
            name = stub_relative_label[1:],
            deps = [actual_full_label],
            srcs = [stub_file],
            tags = tags + ["nolint"],
            **kwargs
        )
