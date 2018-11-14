load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake//tools/skylark:drake_cc.bzl", "drake_cc_library")
load("@drake//tools/skylark:drake_py.bzl", "drake_py_library")

_CC_TEMPLATE = """
#pragma once

/// @file
/// Forwards symbols from `pydrake/common/{name}.h` for backwards
/// compatibility. This header will be deprecated after 2018/12/15.
#include "drake/bindings/pydrake/common/{name}.h"
""".lstrip()

_PY_TEMPLATE = '''
import pydrake.common.{module} as _original
# Include all symbols.
locals().update(_original.__dict__)
# Set documentation explicitly.
__doc__ = """
Forwards symbols from `pydrake.common.{module}` for backwards
compatibility. This module will be deprecated after 2018/12/15.
"""
'''.lstrip()

def util_cc_alias(name):
    src = name + ".h"
    generate_file(
        name = src,
        content = _CC_TEMPLATE.format(name = name),
    )
    drake_cc_library(
        name = name,
        hdrs = [src],
        declare_installed_headers = 0,
        tags = ["nolint"],
        deps = ["//bindings/pydrake/common:" + name],
        visibility = ["//visibility:public"],
    )

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
