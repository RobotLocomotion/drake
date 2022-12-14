# -*- mode: python -*-
# vi: set ft=python :

def _impl(repo_ctx):
    repo_ctx.symlink(
        Label("@drake//tools/workspace/libpfm:package.BUILD.bazel"),
        "BUILD.bazel",
    )

libpfm_repository = repository_rule(
    attrs = {
        "modname": attr.string(default = "libpfm"),
    },
    local = True,
    configure = True,
    implementation = _impl,
)
