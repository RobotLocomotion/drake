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
         `drake/bindings/pydrake/common/{name}.h` instead. This header will \
         be removed after 2019-03-15.
#endif  // _DRAKE_TESTING
""".lstrip()

_PY_TEMPLATE = '''
from pydrake.common.deprecation import _warn_deprecated
import pydrake.common.{module} as _original
# Include all symbols.
locals().update(_original.__dict__)
# Set documentation explicitly.
__doc__ = """
(Deprecated) Forwards symbols from `pydrake.common.{module}` for backwards
compatibility. This module will be removed after 2019-03-15.
"""

_warn_deprecated(__doc__, stacklevel=3)
'''.lstrip()

def util_cc_alias(name):
    src = name + ".h"
    generate_file(
        name = src,
        content = _CC_TEMPLATE.format(name = name),
    )
    name_no_deprecation = "_{}_no_deprecation".format(name)
    drake_cc_library(
        name = name_no_deprecation,
        hdrs = [src],
        declare_installed_headers = 0,
        tags = ["nolint"],
        deps = ["//bindings/pydrake/common:" + name],
    )
    native.cc_library(
        name = name,
        # declare_installed_headers = 0,
        tags = ["nolint", "manual"],
        deps = [name_no_deprecation],
        deprecation = (
            "Please use `//bindings/pydrake/common:{}` instead. " +
            "This will be removed on or after 2019-03-15."
        ).format(name),
        visibility = ["//visibility:public"],
    )
    return ":" + name_no_deprecation

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
