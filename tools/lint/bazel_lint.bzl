load("//tools/skylark:drake_py.bzl", "py_test_isolated")

#------------------------------------------------------------------------------
# Internal helper; set up test given name and list of files. Will do nothing
# if no files given.
def _bazel_lint(name, files):
    if files:
        locations = ["$(locations %s)" % f for f in files]

        py_test_isolated(
            name = name + "_buildifier",
            size = "small",
            srcs = ["@drake//tools/lint:buildifier"],
            data = files,
            args = ["-mode=check"] + locations,
            main = "@drake//tools/lint:buildifier.py",
            tags = ["buildifier", "lint"],
        )

#------------------------------------------------------------------------------
def bazel_lint(
        name = "bazel",
        extra_srcs = None,
        exclude = None):
    """
    Runs ``buildifier -mode=check`` format checker on all Bazel files in the
    current directory.

    Args:
        name: Name prefix of the test (default = "bazel").
        extra_srcs: List of files to lint that would otherwise be missed by the
            default glob pattern for Bazel source code.
        exclude: List to be passed to the skylark glob function for files that
            should not be linted (e.g., vendored files).
    """

    if extra_srcs == None:
        extra_srcs = []
    if exclude == None:
        exclude = []

    _bazel_lint(
        name = name,
        files = native.glob(
            [
                "*.bazel",
                "*.bzl",
                "*.bzlmod",
                "*.BUILD",
                "BUILD",
                "WORKSPACE",
            ],
            exclude = exclude,
            allow_empty = True,
        ) + extra_srcs,
    )
