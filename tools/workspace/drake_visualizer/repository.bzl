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

load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch")
load("@drake//tools/workspace:os.bzl", "determine_os")

def _impl(repository_ctx):
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    if os_result.is_macos:
        archive = "dv-0.1.0-406-g4c3e570a-python-3.9.2-qt-5.15.2-vtk-8.2.0-mac-x86_64.tar.gz"  # noqa
        sha256 = "8a13ffa117167fada851acef8535a42d613b71be2200ea3c7139e9fea05782b8"  # noqa
        python_version = "3.9"
    elif os_result.ubuntu_release == "18.04":
        archive = "dv-0.1.0-406-g4c3e570a-python-3.6.9-qt-5.9.5-vtk-8.2.0-bionic-x86_64-1.tar.gz"  # noqa
        sha256 = "2c477c2f1186cd151710af9a6f50bd3720034ced3c5ed21d977b0a822ac56237"  # noqa
        python_version = "3.6"
    elif os_result.ubuntu_release == "20.04":
        archive = "dv-0.1.0-406-g4c3e570a-python-3.8.2-qt-5.12.8-vtk-8.2.0-focal-x86_64-1.tar.gz"  # noqa
        sha256 = "282438d7fabd72dddc8a9f5b3b7481b6b6ea53e4355f79f5bda1dae6e258d6be"  # noqa
        python_version = "3.8"
    elif os_result.is_manylinux:
        repository_ctx.symlink(
            Label("@drake//tools/workspace/drake_visualizer:package-stub.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        return
    else:
        fail("Operating system is NOT supported {}".format(os_result))

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

    patch(
        repository_ctx,
        patches = [
            Label("@drake//tools/workspace/drake_visualizer:use_drake_lcmtypes.patch"),  # noqa
            Label("@drake//tools/workspace/drake_visualizer:draw_lcm_mesh.patch"),  # noqa
        ],
        patch_args = [
            "--directory=lib/python{}/site-packages/director".format(
                python_version,
            ),
        ],
    )

    repository_ctx.symlink(
        Label("@drake//tools/workspace/drake_visualizer:package.BUILD.bazel"),
        "BUILD.bazel",
    )

drake_visualizer_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
