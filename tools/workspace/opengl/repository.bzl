def _impl(repository_ctx):
    repository_ctx.symlink(
        Label("@drake//tools/workspace/opengl:package.BUILD.bazel"),
        "BUILD.bazel",
    )

opengl_repository = repository_rule(
    implementation = _impl,
)
