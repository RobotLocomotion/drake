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

load(
    "//tools/workspace:metadata.bzl",
    "generate_repository_metadata",
)
load("//tools/workspace/buildifier:version.bzl", "CHECKSUMS", "VERSION")

def _impl(repository_ctx):
    # Determine which binary to fetch.
    os_name = repository_ctx.os.name
    if os_name == "mac os x":
        os_name = "darwin"
    os_arch = repository_ctx.os.arch
    if os_arch == "aarch64":
        os_arch = "arm64"
    filename = "buildifier-{}-{}".format(os_name, os_arch)
    urls = [
        x.format(version = VERSION, filename = filename)
        for x in repository_ctx.attr.mirrors.get("buildifier")
    ]
    sha256 = CHECKSUMS[filename]

    # Fetch the binary from mirrors.
    output = repository_ctx.path("buildifier")
    repository_ctx.download(urls, output, sha256, executable = True)

    # Add the BUILD file.
    repository_ctx.symlink(
        Label("@drake//tools/workspace/buildifier:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Create a summary for S3 mirroring and new_release.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "buildifier",
        version = VERSION,
        downloads = [
            {
                "urls": [
                    x.format(version = VERSION, filename = one_filename)
                    for x in repository_ctx.attr.mirrors.get("buildifier")
                ],
                "sha256": one_checksum,
            }
            for one_filename, one_checksum in CHECKSUMS.items()
        ],
    )

buildifier_repository = repository_rule(
    attrs = {
        "mirrors": attr.string_list_dict(),
    },
    implementation = _impl,
)
