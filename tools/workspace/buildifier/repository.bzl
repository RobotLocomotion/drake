# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads a precompiled version of buildifier and makes it available to the
WORKSPACE.

Example:
    WORKSPACE:
        load("@drake//tools/workspace:mirrors.bzl", "DEFAULT_MIRRORS")
        load("@drake//tools/workspace/buildifier:repository.bzl", "buildifier_repository")  # noqa
        buildifier_repository(name = "foo", mirrors = DEFAULT_MIRRORS)

    BUILD:
        sh_binary(
            name = "foobar",
            srcs = ["bar.sh"],
            data = ["@foo//:buildifier"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:os.bzl", "determine_os")
load(
    "@drake//tools/workspace:metadata.bzl",
    "generate_repository_metadata",
)

def _impl(repository_ctx):
    # Enumerate the possible binaries.  Note that the buildifier binaries are
    # fully statically linked, so the particular distribution doesn't matter,
    # only the kernel.
    version = "5.1.0"
    darwin_urls = [
        x.format(version = version, filename = "buildifier-darwin-amd64")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    darwin_sha256 = "745feb5ea96cb6ff39a76b2821c57591fd70b528325562486d47b5d08900e2e4"  # noqa
    linux_urls = [
        x.format(version = version, filename = "buildifier-linux-amd64")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    linux_sha256 = "52bf6b102cb4f88464e197caac06d69793fa2b05f5ad50a7e7bf6fbd656648a3"  # noqa

    # Choose which binary to use.
    os_result = determine_os(repository_ctx)
    if os_result.is_macos:
        urls = darwin_urls
        sha256 = darwin_sha256
    elif os_result.is_ubuntu or os_result.is_manylinux:
        urls = linux_urls
        sha256 = linux_sha256
    else:
        fail("Operating system is NOT supported {}".format(os_result))

    # Fetch the binary from mirrors.
    output = repository_ctx.path("buildifier")
    repository_ctx.download(urls, output, sha256, executable = True)

    # Add the BUILD file.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/buildifier:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Create a summary file for Drake maintainers.  We need to list all
    # possible binaries so Drake's mirroring scripts will fetch everything.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "manual",
        version = version,
        downloads = [
            {
                "urls": darwin_urls,
                "sha256": darwin_sha256,
            },
            {
                "urls": linux_urls,
                "sha256": linux_sha256,
            },
        ],
    )

buildifier_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
