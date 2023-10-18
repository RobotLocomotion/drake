"""Provides a single point of control for rules_python inside Drake."""

load(
    "@rules_python//python:defs.bzl",
    _py_binary = "py_binary",
    _py_library = "py_library",
    _py_test = "py_test",
)

py_binary = _py_binary

py_library = _py_library

py_test = _py_test
