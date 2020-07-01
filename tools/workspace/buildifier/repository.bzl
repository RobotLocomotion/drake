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
    # Enumerate the possible binaries.
    version = "3.3.0"
    mac_urls = [
        x.format(version = version, filename = "buildifier.mac")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    mac_sha256 = "8d0cae5ec5a79096d014bd3d1533f209aa916aad2b357c3b0130db2c6b1000e5"  # noqa
    ubuntu_urls = [
        x.format(version = version, filename = "buildifier")
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    ubuntu_sha256 = "8a27f46f8a94882ddf37eaf26e5a823a77f04a32c3d72ee2c3d4b5094eb29dc2"  # noqa

    # Choose which binary to use on the current OS.
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_macos:
        urls = mac_urls
        sha256 = mac_sha256
    elif os_result.is_ubuntu:
        urls = ubuntu_urls
        sha256 = ubuntu_sha256
    else:
        fail("Operating system is NOT supported", attr = os_result)

    # Fetch the binary from mirrors.
    output = repository_ctx.path("buildifier")
    repository_ctx.download(urls, output, sha256, executable = True)

    # Add the BUILD file.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/buildifier:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Create a summary file for for Drake maintainers.  We need to list all
    # possible binaries so Drake's mirroring scripts will fetch everything.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "manual",
        version = version,
        downloads = [
            {
                "urls": mac_urls,
                "sha256": mac_sha256,
            },
            {
                "urls": ubuntu_urls,
                "sha256": ubuntu_sha256,
            },
        ],
    )

buildifier_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
