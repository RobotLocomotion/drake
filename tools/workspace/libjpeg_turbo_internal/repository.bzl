def _impl(repo_ctx):
    repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")

libjpeg_turbo_internal_repository = repository_rule(
    attrs = {
        "build_file": attr.label(
            default = "@drake//tools/workspace/libjpeg_turbo_internal:package.BUILD.bazel",  # noqa
        ),
    },
    implementation = _impl,
)
