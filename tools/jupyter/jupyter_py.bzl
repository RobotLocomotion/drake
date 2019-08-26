load("//tools/workspace:generate_file.bzl", "generate_file")
load("@os//:os.bzl", "DISTRIBUTION", "UBUNTU_RELEASE")
load("@python//:version.bzl", "PYTHON_VERSION")
load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_binary",
    "drake_py_test",
)

# Generate file, because we wish to bake the file directly in, and not require
# it be passed as an argument.
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

try:
    from jupyter_bazel import _jupyter_bazel_notebook_main
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
""".lstrip()

def jupyter_py_binary(
        name,
        notebook = None,
        data = [],
        deps = [],
        add_test_rule = 0,
        tags = [],
        test_timeout = None,
        test_flaky = 0,
        **kwargs):
    """Creates a target to run a Jupyter notebook.

    Please see `//tools/jupyter:README.md` for examples.

    @param notebook
        Notebook file to use. Be default, will be `{name}.ipynb`.
    """
    is_unsupported_ubuntu = (
        DISTRIBUTION == "ubuntu" and UBUNTU_RELEASE != "18.04"
    )
    py_major, _ = PYTHON_VERSION.split(".")
    is_unsupported_python = (py_major != "3")
    is_unsupported = (is_unsupported_ubuntu or is_unsupported_python)

    if notebook == None:
        notebook = name + ".ipynb"
    main = "{}_jupyter_py_main.py".format(name)
    tags = tags + ["jupyter"]
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
            "//tools/jupyter:jupyter_bazel_py",
        ],
        # `generate_file` output is still marked as executable :(
        tags = tags + ["nolint"],
        **kwargs
    )
    if add_test_rule:
        target = ":{}".format(name)
        test_tags = tags + ["nolint"]
        if is_unsupported:
            test_tags += ["manual"]
        drake_py_test(
            name = "{}_test".format(name),
            args = ["--test"],
            main = main,
            srcs = [main],
            deps = [target],
            tags = test_tags,
            timeout = test_timeout,
            flaky = test_flaky,
        )
