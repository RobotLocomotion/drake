# -*- mode: python -*-
# vi: set ft=python :

# Between v1.7.0 and v1.7.1, @googlebenchmark added an optional dependency on
# @libpfm. This rule just satisfies the dependency graph at fetch time, without
# providing any actual code for @libpfm.

def _impl(repo_ctx):
    repo_ctx.symlink(
        Label("@drake//tools/workspace/libpfm:package.BUILD.bazel"),
        "BUILD.bazel",
    )

libpfm_repository = repository_rule(
    implementation = _impl,
)
