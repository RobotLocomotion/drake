load("//tools/workspace:generate_file.bzl", "generate_file")
load("@drake_detected_os//:os.bzl", "DISTRIBUTION", "UBUNTU_RELEASE")
load("@python//:version.bzl", "PYTHON_VERSION")
load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_binary",
    "drake_py_test",
)

# Generate file, because we wish to bake the file directly in, and not require
# it be passed as an argument.
# TODO(eric.cousineau): Get rid of Python 2 fail-fast warnings once we remove
# support (#10606).
_JUPYTER_PY_TEMPLATE = """
from __future__ import print_function
import os
import sys
import traceback

_is_unsupported = {is_unsupported}
_unsupported_warning = '''
WARNING:
    Using jupyter_py under Python 2 and/or Ubuntu Xenial is unsupported. You
    may try to resolve these import errors, but there is no guarantee that it
    will work.
'''

def main():
    try:
        from drake.tools.jupyter.jupyter_bazel import (
            _jupyter_bazel_notebook_main)
    except ImportError as e:
        if _is_unsupported:
            traceback.print_exc()
            print(_unsupported_warning, file=sys.stderr)
            sys.exit(1)
        else:
            raise

    cur_dir = os.path.dirname(__file__)
    notebook = {notebook}
    _jupyter_bazel_notebook_main(cur_dir, notebook, sys.argv[1:])


if __name__ == "__main__":
    main()
""".lstrip()

def jupyter_py_binary(
        name,
        notebook = None,
        srcs = None,
        data = [],
        deps = [],
        tags = [],
        add_test_rule = None,
        test_timeout = None,
        test_flaky = 0,
        **kwargs):
    """Creates a target to run a Jupyter notebook.

    Please see `//tools/jupyter:README.md` for examples.

    @param notebook
        Notebook file to use. Be default, will be `{name}.ipynb`.
    """
    if srcs != None:
        fail("srcs is an invalid argument")
    if add_test_rule == None:
        fail("add_test_rule must be explicitly specified")
    is_unsupported_ubuntu = (
        DISTRIBUTION == "ubuntu" and UBUNTU_RELEASE != "18.04"
    )
    py_major, _ = PYTHON_VERSION.split(".")
    is_unsupported_python = (py_major != "3")
    is_unsupported = (is_unsupported_ubuntu or is_unsupported_python)

    if notebook == None:
        notebook = name + ".ipynb"
    main = "{}_jupyter_py_main.py".format(name)

    # Do not lint these generated targets.
    jupyter_tags = tags + ["jupyter", "nolint"]
    generate_file(
        name = main,
        content = _JUPYTER_PY_TEMPLATE.format(
            notebook = repr(notebook),
            is_unsupported = repr(is_unsupported),
        ),
        is_executable = False,
    )
    drake_py_binary(
        name = name,
        srcs = [main],
        main = main,
        data = data + [notebook],
        deps = deps + [
            "@drake//tools/jupyter:jupyter_bazel_py",
        ],
        # `generate_file` output is still marked as executable :(
        tags = jupyter_tags,
        **kwargs
    )
    if add_test_rule:
        target = ":{}".format(name)
        test_tags = jupyter_tags + (["manual"] if is_unsupported else [])
        drake_py_test(
            name = "{}_test".format(name),
            args = ["--test"],
            main = main,
            srcs = [main],
            deps = [target],
            tags = test_tags,
            timeout = test_timeout,
            flaky = test_flaky,
            # Permit `unittest` given that NumPy uses it.
            allow_import_unittest = True,
        )
