"""Provides a single point of control for rules_python inside Drake."""

load(
    "@rules_python//python:defs.bzl",
    _PyInfo = "PyInfo",
    _py_binary = "py_binary",
    _py_library = "py_library",
    _py_test = "py_test",
)

def py_binary(name, **kwargs):
    _py_binary(
        name = name,
        **kwargs
    )

def py_library(name, **kwargs):
    _py_library(
        name = name,
        **kwargs
    )

def py_test(name, **kwargs):
    _py_test(
        name = name,
        **kwargs
    )

PyInfo = _PyInfo
