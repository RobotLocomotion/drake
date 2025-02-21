load("//tools/workspace:workspace_deprecation.bzl", "print_warning")

# Between v1.7.0 and v1.7.1, @googlebenchmark added an optional dependency on
# @libpfm. This rule just satisfies the dependency graph at fetch time, without
# providing any actual code for @libpfm.

def _impl(repo_ctx):
    repo_ctx.symlink(
        Label("@drake//tools/workspace/libpfm:package.BUILD.bazel"),
        "BUILD.bazel",
    )

_libpfm_repository = repository_rule(
    implementation = _impl,
)

def libpfm_repository(
        name,
        _is_drake_self_call = False,
        **kwargs):
    if not _is_drake_self_call:
        print_warning("libpfm_repository")
    _libpfm_repository(
        name = name,
        **kwargs
    )
