"""Provides a single point of control for rules_cc inside Drake.

For all code built by Drake (both first-party code, and built-from-source
externals), we should be using this file's macros. We should never call
`native.cc_foo()` from anywhere other than this file, and any BUILD file
that uses `cc_foo()` rules should load these macros instead of relying
on the (implicitly loaded) `native.cc_foo()` default.

TODO(jwnimmer-tri) Consider writing a linter to check for this, in case
we find that people are unable to manually maintain this invariant.
"""

def cc_binary(**kwargs):
    native.cc_binary(**kwargs)

def cc_import(**kwargs):
    native.cc_import(**kwargs)

def cc_library(**kwargs):
    native.cc_library(**kwargs)

def cc_shared_library(**kwargs):
    native.cc_shared_library(**kwargs)

def cc_test(**kwargs):
    native.cc_test(**kwargs)
