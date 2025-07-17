"""Provides a single point of control for rules_cc inside Drake.

For all code built by Drake (both first-party code, and built-from-source
externals), we should be using this file's macros. Any BUILD file that uses
`cc_foo()` rules should load these macros instead of loading from @rules_cc
directly.
"""

load(
    "@rules_cc//cc:defs.bzl",
    _CcInfo = "CcInfo",
    _cc_binary = "cc_binary",
    _cc_import = "cc_import",
    _cc_library = "cc_library",
    _cc_shared_library = "cc_shared_library",
    _cc_test = "cc_test",
    _objc_library = "objc_library",
)

def cc_binary(**kwargs):
    _cc_binary(**kwargs)

def cc_import(**kwargs):
    _cc_import(**kwargs)

def cc_library(**kwargs):
    _cc_library(**kwargs)

def cc_shared_library(**kwargs):
    _cc_shared_library(**kwargs)

def cc_test(**kwargs):
    _cc_test(**kwargs)

def objc_library(**kwargs):
    _objc_library(**kwargs)

CcInfo = _CcInfo
