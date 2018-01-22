# -*- python -*-

load("//tools/install:install_data.bzl", "install")
load("//tools/skylark:drake_py.bzl", "drake_py_test")

def drake_install_and_test(
        name,
        **kwargs):
    """A wrapper to install and test drake executables."""
    install(
        name = name,
        **kwargs)

    drake_py_test(
        name = name + "_test",
        size = "small",
        # Increase the timeout so that debug builds are successful.
        timeout = "long",
        srcs = [name + "TestCommands.py"],
        data = ["//:install"],
        main = name + "TestCommands.py",
        tags = ["no_everything"],
        deps = ["//tools/install:install_test_helper"],
    )
