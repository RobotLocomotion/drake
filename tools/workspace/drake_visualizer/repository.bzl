# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a precompiled version of drake-visualizer (a subset of
Director, https://git.io/vNKjq) and makes it available to be used as a
dependency of shell scripts.

Archive naming convention:
    dv-<version>-g<commit>-python-<python version>-qt-<qt version>
        -vtk-<vtk version>-<platform>-<arch>[-<rebuild>]

Example:
    WORKSPACE:
        load(
            "@drake//tools/workspace/drake_visualizer:repository.bzl",
            "drake_visualizer_repository",
        )
        drake_visualizer_repository(name = "foo")

    BUILD:
        sh_binary(
            name = "foobar",
            srcs = ["bar.sh"],
            data = ["@foo//:drake_visualizer"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")

# TODO(jamiesnape): Publish scripts used to create binaries. There will be a CI
# job for developers to build new binaries on demand.
def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        archive = "dv-0.1.0-406-g4c3e570a-python-3.9.2-qt-5.15.2-vtk-8.2.0-mac-x86_64.tar.gz"  # noqa
        sha256 = "8a13ffa117167fada851acef8535a42d613b71be2200ea3c7139e9fea05782b8"  # noqa
    elif os_result.ubuntu_release == "18.04":
        archive = "dv-0.1.0-406-g4c3e570a-python-3.6.9-qt-5.9.5-vtk-8.2.0-bionic-x86_64-1.tar.gz"  # noqa
        sha256 = "2c477c2f1186cd151710af9a6f50bd3720034ced3c5ed21d977b0a822ac56237"  # noqa
    elif os_result.ubuntu_release == "20.04":
        archive = "dv-0.1.0-406-g4c3e570a-python-3.8.2-qt-5.12.8-vtk-8.2.0-focal-x86_64-1.tar.gz"  # noqa
        sha256 = "282438d7fabd72dddc8a9f5b3b7481b6b6ea53e4355f79f5bda1dae6e258d6be"  # noqa
    else:
        fail("Operating system is NOT supported", attr = os_result)

    urls = [
        x.format(archive = archive)
        for x in repository_ctx.attr.mirrors.get("director")
    ]
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(
        urls,
        output = root_path,
        sha256 = sha256,
        type = "tar.gz",
    )

    # TODO(jamiesnape): No reason why this file needs to be constructed inline
    # now. Move to a package.BUILD.bazel file, or similar, and symlink.
    file_content = """# -*- python -*-

# DO NOT EDIT: generated by drake_visualizer_repository()

load("@drake//tools/skylark:py.bzl", "py_library")

licenses([
    "notice",  # Apache-2.0 AND BSD-3-Clause AND Python-2.0
    "reciprocal",  # MPL-2.0
    "restricted",  # LGPL-2.1-only AND LGPL-3.0-or-later
    "unencumbered",  # Public-Domain
])

# drake-visualizer has the following non-system dependencies in addition to
# those declared in deps:
#   bot2-lcmgl: LGPL-3.0-or-later
#   ctkPythonConsole: Apache-2.0
#   Eigen: BSD-3-Clause AND MPL-2.0 AND Public-Domain
#   Python: Python-2.0
#   PythonQt: LGPL-2.1-only
#   QtPropertyBrowser: LGPL-2.1-only
# TODO(jamiesnape): Enumerate system dependencies.

py_library(
    name = "drake_visualizer_python_deps",
    deps = [
        "@lcm//:lcm-python",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_py",
        # TODO(eric.cousineau): Expose VTK Python libraries here for Linux.
        "@lcmtypes_robotlocomotion//:lcmtypes_robotlocomotion_py",
    ],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "drake_visualizer",
    srcs = glob([
        "lib/**/*.dylib",
        "lib/**/*.py",
        "lib/**/*.so",
    ]) + [
        "bin/drake-visualizer",
        "share/doc/director/LICENSE.txt",
    ],
    data = [
        ":drake_visualizer_python_deps",
        "@lcm//:libdrake_lcm.so",
        "@vtk",
    ],
    visibility = ["//visibility:public"],
)

load("@drake//tools/install:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = [":drake_visualizer"],
    rename = {
        # Try to 'hide' the binary so that they only use the wrapper script.
        "bin/drake-visualizer": ".drake-visualizer-real",
    },
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

drake_visualizer_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
