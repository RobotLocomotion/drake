def _impl(repo_ctx):
    repo_ctx.symlink(
        Label("@drake//tools/workspace/doxygen_internal:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    if repo_ctx.os.name == "linux":
        # Download and extract Drake's pre-compiled Doxygen binary.
        archive = "doxygen-1.14.0-noble-x86_64.tar.gz"
        url = [
            pattern.format(archive = archive)
            for pattern in repo_ctx.attr.mirrors.get("doxygen")
        ]
        sha256 = "db31e8e25e0c6eae5c90a7a291bf6b65667335c60a7ce69f7d63d1459b2ef8b7"  # noqa
        repo_ctx.download_and_extract(
            url = url,
            sha256 = sha256,
            type = "tar.gz",
        )
    else:
        # On other platforms, documentation builds are not supported, so just
        # provide a dummy binary.
        repo_ctx.file("doxygen", "# Doxygen is not supported on this platform")

doxygen_internal_repository = repository_rule(
    doc = """Provides a :doxygen binary only supported on Ubuntu.""",
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
