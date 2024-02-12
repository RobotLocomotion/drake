"""
Downloads and unpacks a precompiled version of drake-visualizer (a subset of
Director, https://git.io/vNKjq) and makes it available to be used as a
dependency of shell scripts.

The drake_visualizer binaries are *only* provided for Ubuntu 20.04 Focal.

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
load("//tools/workspace:execute.bzl", "execute_and_return")

def _impl(repository_ctx):
    release = execute_and_return(repository_ctx, ["lsb_release", "-sr"])
    if release.error or release.stdout.strip() != "20.04":
        repository_ctx.file("defs.bzl", "ENABLED = False")
        repository_ctx.symlink(
            Label("@drake//tools/workspace/drake_visualizer:package-stub.BUILD.bazel"),  # noqa
            "BUILD.bazel",
        )
        return

    repository_ctx.file("defs.bzl", "ENABLED = True")

    archive = "dv-0.1.0-406-g4c3e570a-python-3.8.10-qt-5.12.8-vtk-8.2.0-focal-x86_64-5.tar.gz"  # noqa
    sha256 = "ddf85e332c0b7be8b13e81e6627a94f09a70517b7f82d7a49e50367b20cede87"  # noqa
    repository_ctx.download_and_extract(
        [
            x.format(archive = archive)
            for x in repository_ctx.attr.mirrors.get("director")
        ],
        output = repository_ctx.path(""),
        sha256 = sha256,
        type = "tar.gz",
    )

    patch(
        repository_ctx,
        patches = [
            Label("@drake//tools/workspace/drake_visualizer:patches/use_drake_lcmtypes.patch"),  # noqa
            Label("@drake//tools/workspace/drake_visualizer:patches/draw_lcm_mesh.patch"),  # noqa
            Label("@drake//tools/workspace/drake_visualizer:patches/warn_gltf.patch"),  # noqa
        ],
        patch_args = [
            "--directory=lib/python3.8/site-packages/director",
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
