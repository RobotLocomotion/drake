load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/skylark:drake_cc.bzl", "drake_cc_library")
load("@drake//tools/skylark:drake_py.bzl", "drake_py_library")

_CC_TEMPLATE = r"""
#pragma once

/// @file
/// (Deprecated) Forwards symbols from `drake/bindings/pydrake/common/{name}.h`
/// for backwards compatibility. This header will be removed after 2019-03-15.
#include "drake/bindings/pydrake/common/{name}.h"

#ifndef _DRAKE_TESTING
#warning This header is deprecated. Please use \
         drake/bindings/pydrake/common/{name}.h instead. This header will \
         be removed on or after 2019-03-15.
#endif  // _DRAKE_TESTING
""".lstrip()

_PY_TEMPLATE = '''
"""
Warning:
    Deprecated. Please use ``pydrake.common.{module}`` instead. This module
    will be removed on or after 2019-03-15.
"""

from pydrake.common.deprecation import _warn_deprecated
import pydrake.common.{module} as _original
# Include all symbols, but save `__doc__`.
_old_doc = __doc__
locals().update(_original.__dict__)
__doc__ = _old_doc
_warn_deprecated(__doc__, stacklevel=3)
'''.lstrip()

def util_cc_alias(name):
    """
    Generates a header with a deprecation message, and generates two Bazel
    targets:
    1. a private target without a deprecation message, and
    2. a public target with a deprecation message.
    @return The private target name for testing.
    """
    src = name + ".h"
    generate_file(
        name = src,
        content = _CC_TEMPLATE.format(name = name),
    )
    public_target_with_deprectation = name
    private_target_without_deprecation = "_{}".format(name)
    drake_cc_library(
        name = private_target_without_deprecation,
        hdrs = [src],
        declare_installed_headers = 0,
        tags = ["nolint"],
        deps = ["//bindings/pydrake/common:" + name],
    )
    native.cc_library(
        name = public_target_with_deprectation,
        tags = ["nolint", "manual"],
        deps = [private_target_without_deprecation],
        deprecation = (
            "Please use `//bindings/pydrake/common:{}` instead. " +
            "This will be removed on or after 2019-03-15."
        ).format(name),
        visibility = ["//visibility:public"],
    )
    return ":" + private_target_without_deprecation

def util_py_alias(name):
    if not name.endswith("_py"):
        fail("Invalid name")
    module = name[:-3]
    src = module + ".py"
    generate_file(
        name = src,
        content = _PY_TEMPLATE.format(module = module),
    )
    drake_py_library(
        name = name,
        srcs = [src],
        tags = ["nolint"],
        deps = [
            ":module_py",
            "//bindings/pydrake/common:" + name,
        ],
    )
