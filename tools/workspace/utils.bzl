# -*- mode: python -*-
# vi: set ft=python :

load("@drake//tools/workspace:execute.bzl", "execute_or_fail")

PATCH_ATTRS = {
    "patches": attr.label_list(),
    "patch_cmds": attr.string_list(),
    "patch_tool": attr.string(default = "patch"),
    "patch_args": attr.string_list(default = ["-p0"]),
}
"""List of attributes to pass to the function patch(repository_ctx) in
@bazel_tools//tools/build_defs/repo:utils.bzl.
"""

def delete_bazel_files(repository_ctx):
    """Delete any existing BUILD, BUILD.bazel, or WORKSPACE files from the
    archive. Repository rules that use this function should add BAZEL_SH to
    the value of their environ argument.

    Args:
        repository_ctx: context of a Bazel repository rule.
    """
    bash = repository_ctx.os.environ.get("BAZEL_SH", "bash")
    execute_or_fail(repository_ctx, [bash, "-c", """
        set -euxo pipefail
        find . \
            -name BUILD -print0 -o \
            -name BUILD.bazel -print0 -o \
            -name WORKSPACE -print0 |
            xargs -t -n1 -0 -I{} mv {} {}.upstream-ignored
        """])

def sha256_or_zero(sha256):
    """Fallback to an incorrect default value of SHA-256 hash to prevent the
    hash check from being disabled, but allow the first download attempt of an
    archive to fail and print the correct hash.

    Args:
        sha256: expected SHA-256 hash of the archive to be downloaded.
    """
    if not sha256:
        sha256 = "0" * 64

    return sha256
