"""Provides a single point of control for rules_cc inside Drake."""

def cc_binary(**kwargs):
    native.cc_binary(**kwargs)

def cc_import(**kwargs):
    native.cc_input(**kwargs)

def cc_library(**kwargs):
    native.cc_library(**kwargs)

def cc_shared_library(**kwargs):
    native.cc_shared_library(**kwargs)

def cc_test(**kwargs):
    native.cc_test(**kwargs)
