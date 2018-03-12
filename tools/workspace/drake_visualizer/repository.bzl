# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a precompiled version of drake-visualizer (a subset of
Director, https://git.io/vNKjq) and makes it available to be used as a
dependency of shell scripts.

Archive naming convention:
    dv-<version>-g<commit>-qt-<qt version>-vtk-<vtk version>-<platform>-<arch>

Build configuration:
    BUILD_SHARED_LIBS=OFF
    CMAKE_BUILD_TYPE=Release
    DD_QT_VERSION=5
    USE_EXTERNAL_INSTALL=ON
    USE_LCM=ON
    USE_SYSTEM_VTK=ON

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
        archive = "dv-0.1.0-282-g1a968bd-qt-5.10.0-vtk-8.1.0-mac-x86_64.tar.gz"
        sha256 = "2dee827345d5696b0097024a24be710a4aa9703480860c93ce528771372b0aab"  # noqa
    elif os_result.ubuntu_release == "16.04":
        archive = "dv-0.1.0-282-g1a968bd-qt-5.5.1-vtk-8.1.0-xenial-x86_64.tar.gz"  # noqa
        sha256 = "9612d2d923280b0d76aed3f44dce575e0962e3c72ecdf0b8799532a5df203ebd"  # noqa
    else:
        fail("Operating system is NOT supported", attr = os_result)

    urls = [
        x.format(archive = archive)
        for x in repository_ctx.attr.mirrors.get("director")
    ]
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(urls, root_path, sha256 = sha256)

    file_content = """
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
        "lib/libPythonQt.*",
        "lib/libddApp.*",
        "lib/python2.7/site-packages/director/**/*.py",
        "lib/python2.7/site-packages/director/**/*.so",
        "lib/python2.7/site-packages/urdf_parser_py/**/*.py",
    ]) + [
        "bin/drake-visualizer",
        "share/doc/director/LICENSE.txt",
    ],
    data = [
        ":drake_visualizer_python_deps",
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

drake_visualizer_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
