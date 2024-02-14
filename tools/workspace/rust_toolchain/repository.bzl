load("//tools/workspace/rust_toolchain:lock/archives.bzl", "ARCHIVES")
load("//tools/workspace:metadata.bzl", "generate_repository_metadata")

def _rust_toolchain_downloads_impl(repo_ctx):
    # TODO(jwnimmer-tri) It's not clear to me yet whether we really need to
    # mirror the (giant) rust compiler binaries. For now, we'll leave the
    # mirroring infrastructure in place but not actually employ it.
    #
    # When/if this changes, it may be necessary to manually enumerate these
    # repositories in tools/workspace/metadata.py.
    downloads = json.decode(repo_ctx.attr.downloads)
    for item in downloads:
        repo_ctx.download_and_extract(**item)
    repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")
    generate_repository_metadata(
        repo_ctx,
        repository_rule_type = "rust_toolchain_downloads",
        downloads = downloads,
    )

rust_toolchain_downloads = repository_rule(
    implementation = _rust_toolchain_downloads_impl,
    doc = "Downloads all files from `downloads` then adds the `build_file`.",
    attrs = {
        "mirrors": attr.string_list_dict(mandatory = True),
        "build_file": attr.label(mandatory = True),
        "downloads": attr.string(
            mandatory = True,
            doc = (
                "JSON-encoded string containing a list. Each item in the " +
                "list is a dict of kwargs to download_and_extract."
            ),
        ),
    },
)

def rust_toolchain_repositories(*, mirrors, excludes = []):
    """Adds multiple repository rules pointing to rust-lang.org downloads.
    Refer to README.md for details.
    """
    for kwargs in ARCHIVES:
        if kwargs["name"] not in excludes:
            rust_toolchain_downloads(
                mirrors = mirrors,
                **kwargs
            )

def register_rust_toolchains():
    """Registers all of the Rust toolchains that rust_toolchain_repositories()
    knows how to downloaded. Refer to README.md for details. If you've excluded
    certain downloads via `excludes = ...` above, then don't use this function.
    """
    for name in [kwargs["name"] for kwargs in ARCHIVES]:
        if name.startswith("rust_") and name.endswith("__stable"):
            native.register_toolchains("@{}//:toolchain".format(name))
