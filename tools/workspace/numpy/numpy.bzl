# -*- mode: python -*-
# vi: set ft=python :

"""
Finds local system NumPy headers and makes them available to be used as a
C/C++ dependency.

Example:
    WORKSPACE:
        load("//tools/workspace/numpy:numpy.bzl", "numpy_repository")
        numpy_repository(
            name = "foo",
            python_version = "2.7",
        )

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:numpy"],
            srcs = ["bar.cc"],
        )

Arguments:
    name: A unique name for this rule.
    python_version: The version of Python for which NumPy headers are to be
                    found.
"""

def _impl(repository_ctx):
    python = repository_ctx.which("python{}".format(
        repository_ctx.attr.python_version))

    if not python:
        fail("Could NOT find python{}".format(repository_ctx.attr.version))

    result = repository_ctx.execute([
        python,
        "-c",
        "; ".join([
            "from __future__ import print_function",
            "import numpy",
            "print(numpy.get_include())",
        ]),
    ])

    if result.return_code != 0:
        fail("Could NOT determine NumPy include", attr = result.stderr)

    source = repository_ctx.path(result.stdout.strip())
    destination = repository_ctx.path("include")
    repository_ctx.symlink(source, destination)

    file_content = """
cc_library(
    name = "numpy",
    hdrs = glob(["include/**"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
    """

    repository_ctx.file("BUILD", content = file_content, executable = False)

numpy_repository = repository_rule(
    _impl,
    attrs = {"python_version": attr.string(default = "2.7")},
    local = True,
)
