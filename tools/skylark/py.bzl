"""Provides a single point of control for rules_python inside Drake."""

load(
    "@rules_python//python:defs.bzl",
    _PyInfo = "PyInfo",
    _py_binary = "py_binary",
    _py_library = "py_library",
    _py_test = "py_test",
)

# All of Drake's Python code should depend on our requirements.txt pins, so we
# add it as a data dependency to every python rule.

def _add_requirements(data):
    return (data or []) + ["@drake//tools/workspace/python:requirements"]

def py_binary(name, *, data = None, **kwargs):
    _py_binary(
        name = name,
        data = _add_requirements(data),
        **kwargs
    )

def py_library(name, *, data = None, **kwargs):
    _py_library(
        name = name,
        data = _add_requirements(data),
        **kwargs
    )

def py_test(name, *, data = None, **kwargs):
    _py_test(
        name = name,
        data = _add_requirements(data),
        **kwargs
    )

PyInfo = _PyInfo
