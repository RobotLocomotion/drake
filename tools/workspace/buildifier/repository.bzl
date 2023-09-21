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

load("//tools/workspace:os.bzl", "determine_os")
load(
    "//tools/workspace:metadata.bzl",
    "generate_repository_metadata",
)

def _impl(repository_ctx):
    # Enumerate the possible binaries.  Note that the buildifier binaries are
    # fully statically linked, so the particular distribution doesn't matter,
    # only the kernel.
    #
    # To update this, each artifact needs to be downloaded and its checksum
    # computed manually.  See tools/workspace/mirrors.bzl for the canonical
    # URL.
    version = "v6.3.3"
    darwin_urls = [
        x.format(version = version, filename = "buildifier-darwin-amd64")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    darwin_sha256 = "3c36a3217bd793815a907a8e5bf81c291e2d35d73c6073914640a5f42e65f73f"  # noqa
    linux_urls = [
        x.format(version = version, filename = "buildifier-linux-amd64")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    linux_sha256 = "42f798ec532c58e34401985043e660cb19d5ae994e108d19298c7d229547ffca"  # noqa

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
