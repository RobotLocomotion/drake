# -*- python -*-

def _impl(repository_ctx):
    # On Ubuntu, we use python3-pygame from apt.  On macOS, we do not offer a
    # working @pygame repostory.  Either way, the BUILD is the same.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/pygame_py:package-stub.BUILD.bazel"),
        "BUILD.bazel",
    )

pygame_py_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(
            doc = """Dictionary such as the provided DEFAULT_MIRRORS from
            @drake//tools/workspace:mirrors.bzl""",
            mandatory = True,
            allow_empty = False,
        ),
    },
    implementation = _impl,
)
