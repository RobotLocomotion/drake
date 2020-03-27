load("//tools/workspace:generate_file.bzl", "generate_file")
load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_binary",
    "drake_py_test",
)

# Generate file, because we wish to bake the file directly in, and not require
# it be passed as an argument.
_JUPYTER_PY_TEMPLATE = """
import os
import sys

from drake.tools.jupyter.jupyter_bazel import _jupyter_bazel_notebook_main


def main():
    cur_dir = os.path.dirname(__file__)
    notebook = {notebook}
    _jupyter_bazel_notebook_main(cur_dir, notebook, sys.argv[1:])


if __name__ == "__main__":
    main()
""".lstrip()

def drake_jupyter_py_binary(
        name,
        notebook = None,
        srcs = None,
        data = [],
        deps = [],
        tags = [],
        add_test_rule = None,
        test_timeout = None,
        # TODO(eric.cousineau): Reset default flaky value to False once #12536
        # is resolved.
        test_flaky = True,
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
    if notebook == None:
        notebook = name + ".ipynb"
    main = "{}_jupyter_py_main.py".format(name)

    # Do not lint these generated targets.
    jupyter_tags = tags + ["jupyter", "nolint"]
    generate_file(
        name = main,
        content = _JUPYTER_PY_TEMPLATE.format(
            notebook = repr(notebook),
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
        test_tags = jupyter_tags
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
