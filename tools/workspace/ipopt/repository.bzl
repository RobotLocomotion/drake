def _impl(repo_ctx):
    repo_ctx.symlink(
        Label("@drake//tools/workspace/ipopt:package.BUILD.bazel"),
        "BUILD.bazel",
    )

ipopt_repository = repository_rule(
    local = True,
    implementation = _impl,
)
