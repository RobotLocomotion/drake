def _impl(repo_ctx):
    repo_ctx.symlink(
        Label("@drake//tools/workspace/doxygen:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    if repo_ctx.os.name == "linux":
        # Download and extract Drake's pre-compiled Doxygen binary.
        archive = "doxygen-1.8.15-focal-x86_64.tar.gz"
        url = [
            pattern.format(archive = archive)
            for pattern in repo_ctx.attr.mirrors.get("doxygen")
        ]
        sha256 = "3c4886763ec27e1797b0fd5bfe576602f5e408649c0e282936e17bde5c7ed7e6"  # noqa
        repo_ctx.download_and_extract(
            url = url,
            sha256 = sha256,
            type = "tar.gz",
        )
    else:
        # On other platforms, documentation builds are not supported, so just
        # provide a dummy binary.
        repo_ctx.file("doxygen", "# Doxygen is not supported on this platform")

doxygen_repository = repository_rule(
    doc = """Provides a @doxygen//:doxygen binary only supported on Ubuntu.""",
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
