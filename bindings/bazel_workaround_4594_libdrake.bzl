# -*- python -*-

def get_bazel_workaround_4594_libdrake_package_info():
    """Only to be used by //bindings:bazel_workaround_4594_libdrake_py.

    For more information, see `pydrake/__init__.py`.
    """
    return struct(py_imports = ["../.."])
