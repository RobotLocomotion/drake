load("@bazel_tools//tools/build_defs/repo:utils.bzl", "patch")
load("//tools/workspace/crate_universe:lock/archives.bzl", "ARCHIVES")
load("//tools/workspace:metadata.bzl", "generate_repository_metadata")

def _add_mirrors(*, urls, mirrors):
    # The input `urls` will be a singleton list like this:
    # [
    #   "https://crates.io/api/v1/crates/amd/0.2.2/download",
    # ]
    #
    # Our goal is to make it into a multi-URL list like this:
    # [
    #   "https://crates.io/api/v1/crates/amd/0.2.2/download",
    #   "https://drake-mirror.csail.mit.edu/crates.io/amd/0.2.2/download",
    #   "https://s3.amazonaws.com/drake-mirror/crates.io/amd/0.2.2/download",
    # ]

    # Parse the "https://crates.io/..." to find the suffix we want.
    if len(urls) != 1:
        fail("Expected exactly one crate URL, got: " + str(urls))
    (default_url,) = urls
    middle = "/api/v1/crates/"
    tokens = default_url.split(middle)
    if len(tokens) != 2:
        fail("Failed to match " + middle + " in URL " + default_url)
    (_, archive) = tokens

    # Substitute the {default_url} or {archive} into the mirror pattern(s).
    result = []
    for pattern in mirrors["crate_universe"]:
        if pattern == "{default_url}":
            result.append(default_url)
        else:
            result.append(pattern.format(archive = archive))
    return result

def _create_http_archive_impl(repo_ctx):
    urls = _add_mirrors(
        urls = repo_ctx.attr.urls,
        mirrors = repo_ctx.attr.mirrors,
    )
    sha256 = repo_ctx.attr.sha256
    strip_prefix = repo_ctx.attr.strip_prefix
    type = repo_ctx.attr.type
    repo_ctx.download_and_extract(
        url = urls,
        sha256 = sha256,
        stripPrefix = strip_prefix,
        type = type,
    )
    if repo_ctx.attr.patches:
        patch(repo_ctx)
    repo_ctx.symlink(repo_ctx.attr.build_file, "BUILD.bazel")
    generate_repository_metadata(
        repo_ctx,
        repository_rule_type = "crate_universe",
        urls = urls,
        sha256 = sha256,
    )

crate_http_archive = repository_rule(
    implementation = _create_http_archive_impl,
    attrs = {
        "build_file": attr.label(
            mandatory = True,
        ),
        "mirrors": attr.string_list_dict(
            mandatory = True,
        ),
        "patches": attr.label_list(),
        "patch_tool": attr.string(default = "patch"),
        "patch_args": attr.string_list(default = ["-p0"]),
        "sha256": attr.string(
            mandatory = True,
        ),
        "strip_prefix": attr.string(
            mandatory = True,
        ),
        "type": attr.string(
            mandatory = True,
        ),
        "urls": attr.string_list(
            mandatory = True,
        ),
    },
)

def crate_universe_repositories(*, mirrors, excludes = []):
    # This dependency is part of a "cohort" defined in
    # drake/tools/workspace/new_release.py.  When practical, all members of
    # this cohort should be updated at the same time.
    #
    # Metadata for this repository is additionally defined in
    # drake/tools/workspace/metadata.py.
    for kwargs in ARCHIVES:
        if kwargs["name"] not in excludes:
            crate_http_archive(
                mirrors = mirrors,
                **kwargs
            )
