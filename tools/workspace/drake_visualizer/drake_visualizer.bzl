# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a precompiled version of drake-visualizer (a subset of
Director) and makes it available to be used as a dependency of shell scripts.

Archive naming convention:
    dv-<version>-g<commit>-qt-<qt version>-vtk-<vtk version>-<platform>-<arch>

Build configuration:
    BUILD_SHARED_LIBS=OFF
    CMAKE_BUILD_TYPE=Release
    DD_QT_VERSION=5
    USE_LCM=ON
    USE_SYSTEM_VTK=ON

Example:
    WORKSPACE:
        load(
            "//tools/workspace/drake_visualizer:drake_visualizer.bzl",
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
        archive = "dv-0.1.0-173-g6e49220-qt-5.9.1-vtk-8.0.1-mac-x86_64.tar.gz"
        sha256 = "65b78914327c82bb8fd7cf2182dedc2a45edeafc44fc229415528ee2180bf9a4"  # noqa
    elif os_result.ubuntu_release == "16.04":
        archive = "dv-0.1.0-173-g6e49220-qt-5.5.1-vtk-8.0.1-xenial-x86_64.tar.gz"  # noqa
        sha256 = "57ebe3cef758b42bdc1affb50e371a1e5224e73e3c2ebe25dcbba7697b66d24d"  # noqa
    else:
        fail("Operating system is NOT supported", attr = os_result)

    urls = [
        "https://drake-packages.csail.mit.edu/director/{}".format(archive),
        "https://s3.amazonaws.com/drake-packages/director/{}".format(archive),
    ]
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(urls, root_path, sha256 = sha256)

    file_content = """
filegroup(
    name = "drake_visualizer",
    srcs = glob([
        "lib/libPythonQt.*",
        "lib/libddApp.*",
        "lib/python2.7/dist-packages/director/**/*.py",
        "lib/python2.7/dist-packages/director/**/*.so",
        "lib/python2.7/dist-packages/urdf_parser_py/**/*.py",
    ]) + [
        "bin/drake-visualizer",
        "share/doc/director/LICENSE.txt",
    ],
    data = [
        "@lcm//:lcm-python",
        "@lcmtypes_bot2_core//:lcmtypes_bot2_core_py",
        "@lcmtypes_robotlocomotion//:lcmtypes_robotlocomotion_py",
        "@vtk",
    ],
    visibility = ["//visibility:public"],
)

load("@drake//tools/install:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = [":drake_visualizer"],
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file("BUILD", content = file_content, executable = False)

drake_visualizer_repository = repository_rule(implementation = _impl)
