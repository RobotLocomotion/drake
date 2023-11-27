def _impl(repository_ctx):
    repository_ctx.symlink(
        Label("@drake//tools/workspace/opencl:package.BUILD.bazel"),
        "BUILD.bazel",
    )

opencl_repository = repository_rule(
    implementation = _impl,
)
