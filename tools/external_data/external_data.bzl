# -*- python -*-

def external_data_stub_test():
    # Define stub test using upstream package's file.
    file = "@drake//tools/external_data/bazel_external_data:stub_test.py"
    native.py_test(
        name = "stub_test",
        srcs = [file],
        main = file,
    )
