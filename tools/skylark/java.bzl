"""Provides a single point of control for rules_java inside Drake."""

load(
    "@rules_java//java:java_binary.bzl",
    _java_binary = "java_binary",
)
load(
    "@rules_java//java:java_import.bzl",
    _java_import = "java_import",
)
load(
    "@rules_java//java:java_library.bzl",
    _java_library = "java_library",
)
load(
    "@rules_java//java/common:java_info.bzl",
    _JavaInfo = "JavaInfo",
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

JavaInfo = _JavaInfo
