load("//tools/workspace:metadata.bzl", "generate_repository_metadata")

def _impl(repo_ctx):
    repository = repo_ctx.attr.repository
    commit = repo_ctx.attr.commit
    platform = repo_ctx.attr.platform
    sha256 = repo_ctx.attr.sha256
    mirrors = repo_ctx.attr.mirrors

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/doxygen_internal:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Provide upgrade details to our new_release script.
    generate_repository_metadata(
        repo_ctx,
        repository_rule_type = "github",
        upgrade_advice = """
        See tools/workspace/doxygen_internal/README.md to upgrade.
        """,
        repository = repository,
        commit = commit,
        # Opt out of mirroring the Doxygen source to S3. mirror_to_s3 globs all
        # repository metadata when looking for sources to mirror, but that
        # won't (and shouldn't) work here, because Doxygen comes from S3 itself.
        mirror_to_s3 = False,
    )

    if repo_ctx.os.name == "linux":
        # Parse the version string from the commit.
        version = commit.removeprefix("Release_").replace("_", ".")

        # Download and extract Drake's pre-compiled Doxygen binary.
        archive = "doxygen-{}-{}-x86_64.tar.gz".format(version, platform)
        url = [
            pattern.format(archive = archive)
            for pattern in mirrors.get("doxygen")
        ]
        repo_ctx.download_and_extract(
            url = url,
            sha256 = sha256,
            type = "tar.gz",
        )
    else:
        # On other platforms, documentation builds are not supported, so just
        # provide a dummy binary.
        repo_ctx.file("doxygen", "# Doxygen is not supported on this platform")

_doxygen_internal_repository_impl = repository_rule(
    doc = """Provides a :doxygen binary only supported on Ubuntu.""",
    implementation = _impl,
    attrs = {
        "repository": attr.string(
            mandatory = True,
        ),
        "commit": attr.string(
            mandatory = True,
        ),
        "platform": attr.string(
            mandatory = True,
        ),
        "sha256": attr.string(
            mandatory = False,
            default = "0" * 64,
        ),
        "mirrors": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
    },
)

def doxygen_internal_repository(
        name,
        mirrors = None):
    _doxygen_internal_repository_impl(
        name = name,
        repository = "doxygen/doxygen",
        commit = "Release_1_15_0",
        platform = "noble",
        sha256 = "127f10a70aa3bf526f07f27d4c2f72c105ef72a32ba48a136320af09935d06f6",  # noqa
        mirrors = mirrors,
    )
