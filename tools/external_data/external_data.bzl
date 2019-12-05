# -*- python -*-

load("//tools/skylark:py.bzl", "py_test")

def external_data_stub_test():
    # Define stub test using upstream package's file.
    file = "@drake//tools/external_data/bazel_external_data:stub_test.py"
    py_test(
        name = "stub_test",
        srcs = [file],
        main = file,
    )
