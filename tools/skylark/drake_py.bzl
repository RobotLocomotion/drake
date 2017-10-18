# -*- python -*-

def drake_py_library(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    native.py_library(
        name = name,
        **kwargs)

def drake_py_binary(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    native.py_binary(
        name = name,
        **kwargs)

def drake_py_test(
        name,
        **kwargs):
    """A wrapper to insert Drake-specific customizations."""
    native.py_test(
        name = name,
        **kwargs)
