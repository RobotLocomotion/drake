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
    DD_QT_VERSION=4 (Trusty)
    DD_QT_VERSION=5 (Mac and Xenial)
    USE_LCM=ON
    USE_SYSTEM_VTK=ON

Example:
    WORKSPACE:
        load("//tools:drake_visualizer.bzl", "drake_visualizer_repository")
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

# TODO(jamiesnape): Publish scripts used to create binaries. There will be a CI
# job for developers to build new binaries on demand.
def _impl(repository_ctx):
    if repository_ctx.os.name == "mac os x":
        archive = "dv-0.1.0-173-g6e49220-qt-5.9.1-vtk-8.0.1-mac-x86_64.tar.gz"
        sha256 = "65b78914327c82bb8fd7cf2182dedc2a45edeafc44fc229415528ee2180bf9a4"  # noqa
    elif repository_ctx.os.name == "linux":
        sed = repository_ctx.which("sed")

        if not sed:
            fail("Could NOT determine Linux distribution information because" +
                 " sed is missing")

        result = repository_ctx.execute([
            sed,
            "-n",
            "/^\(NAME\|VERSION_ID\)=/{s/[^=]*=//;s/\"//g;p}",
            "/etc/os-release"])

        if result.return_code != 0:
            fail("Could NOT determine Linux distribution information",
                 attr = result.stderr)

        distro = [l.strip() for l in result.stdout.strip().split("\n")]
        distro = " ".join(distro)

        if distro == "Ubuntu 14.04":
            archive = "dv-0.1.0-173-g6e49220-qt-4.8.6-vtk-8.0.1-trusty-x86_64.tar.gz"  # noqa
            sha256 = "a9b03955cc22803f418fc712d98b3b0f83411d480ed63c6544e6ddfa141e92d5"  # noqa
        elif distro == "Ubuntu 16.04":
            archive = "dv-0.1.0-173-g6e49220-qt-5.5.1-vtk-8.0.1-xenial-x86_64.tar.gz"  # noqa
            sha256 = "57ebe3cef758b42bdc1affb50e371a1e5224e73e3c2ebe25dcbba7697b66d24d"  # noqa
        else:
            fail("Linux distribution is NOT supported", attr = distro)
    else:
        fail("Operating system is NOT supported",
             attr = repository_ctx.os.name)

    url = "https://d2mbb5ninhlpdu.cloudfront.net/director/{}".format(archive)
    root_path = repository_ctx.path("")

    repository_ctx.download_and_extract(url, root_path, sha256 = sha256)

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

load("@drake//tools:install.bzl", "install_files")
install_files(
    name = "install",
    dest = ".",
    files = [":drake_visualizer"],
    visibility = ["//visibility:public"],
)
"""

    repository_ctx.file("BUILD", content = file_content, executable = False)

drake_visualizer_repository = repository_rule(implementation = _impl)
