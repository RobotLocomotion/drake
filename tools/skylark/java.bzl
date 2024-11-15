"""Provides a single point of control for rules_java inside Drake."""

load(
    "@rules_java//java:defs.bzl",
    _java_binary = "java_binary",
    _java_import = "java_import",
    _java_library = "java_library",
)

def java_binary(name, **kwargs):
    _java_binary(
        name = name,
        **kwargs
    )

def java_import(name, **kwargs):
    _java_import(
        name = name,
        **kwargs
    )

def java_library(name, **kwargs):
    _java_library(
        name = name,
        **kwargs
    )
