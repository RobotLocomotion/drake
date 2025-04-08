load(
    "//tools/skylark:drake_py.bzl",
    "drake_py_binary",
    "drake_py_test",
)
load("//tools/workspace:generate_file.bzl", "generate_file")

# Generate file to bake file path in rather than require it as an argument.
_JUPYTER_PY_TEMPLATE = """
import sys

from jupyter_bazel import _jupyter_bazel_notebook_main


def main():
    _jupyter_bazel_notebook_main({notebook_respath}, sys.argv[1:])


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
        test_tags = [],
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

    notebook_respath = "drake/{}/{}".format(
        native.package_name(),
        notebook,
    ).replace("//", "/")

    # Do not lint these generated targets.
    jupyter_tags = tags + ["jupyter", "nolint"]
    generate_file(
        name = main,
        content = _JUPYTER_PY_TEMPLATE.format(
            notebook_respath = repr(notebook_respath),
        ),
        is_executable = False,
    )
    drake_py_binary(
        name = name,
        srcs = [main],
        main = main,
        data = data + [notebook],
        deps = depset(deps + [
            "@rules_python//python/runfiles",
            "@drake//tools/jupyter:jupyter_bazel_py",
        ]).to_list(),
        # `generate_file` output is still marked as executable :(
        tags = jupyter_tags,
        **kwargs
    )
    if add_test_rule:
        target = ":{}".format(name)
        drake_py_test(
            name = "{}_test".format(name),
            args = ["--test"],
            main = main,
            srcs = [main],
            deps = [target],
            tags = jupyter_tags + test_tags,
            timeout = test_timeout,
            flaky = test_flaky,
            # Permit `unittest` given that NumPy uses it.
            allow_import_unittest = True,
        )
