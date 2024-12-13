"""Provides a single point of control for rules_shell inside Drake."""

load("@rules_shell//shell:sh_binary.bzl", _sh_binary = "sh_binary")
load("@rules_shell//shell:sh_test.bzl", _sh_test = "sh_test")

def sh_binary(name, **kwargs):
    _sh_binary(
        name = name,
        **kwargs
    )

def sh_test(name, **kwargs):
    _sh_test(
        name = name,
        **kwargs
    )
