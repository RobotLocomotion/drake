# -*- python -*-

def _impl(repository_ctx):
    print("WARNING: the @pygame_py repository in drake is deprecated, it no " +
          "longer not provides `pygame` for bionic, use `apt` to install " +
          "`python3-pygame` or `pip`.  This stub will be removed on or " +
          "after 2022-07-15.")

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
