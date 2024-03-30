load("//tools/workspace:execute.bzl", "execute_or_fail")
load("//tools/workspace:metadata.bzl", "generate_repository_metadata")

def _impl(repo_ctx):
    version = repo_ctx.attr.version
    mirrors = repo_ctx.attr.mirrors
    sha256 = repo_ctx.attr.sha256
    version_major, version_minor, _ = version.split(".", 2)

    # Convert the Bazel's spelling of platform strings to Mosek spellings.
    os_name = repo_ctx.os.name  # "linux" or "mac os x"
    os_arch = repo_ctx.os.arch  # "aarch64" or "amd64" or "x86_64"
    if os_name == "mac os x":
        os_name = "osx"
    if os_arch in ["amd64", "x86_64"]:
        os_arch = "64x86"
    mosek_platform = os_name + os_arch
    basename = "mosektools{}.tar.bz2".format(mosek_platform)

    # Download and extract the MOSEK™ binary release. The only parts we want to
    # keep are the EULA document and the tools/platform/{os_arch} directory.
    urls = [
        url.format(path = "stable/{}/{}".format(version, basename))
        for url in mirrors["mosek"]
    ]
    checksum = sha256.get(basename)
    base_prefix = "mosek/{}.{}".format(version_major, version_minor)
    strip_prefix = "{}/tools/platform/{}".format(base_prefix, mosek_platform)
    rename_files = {
        "{}/mosek-eula.pdf".format(base_prefix): "{}/mosek-eula.pdf".format(strip_prefix),  # noqa
    }
    repo_ctx.download_and_extract(
        urls,
        output = repo_ctx.path(""),
        sha256 = checksum,
        rename_files = rename_files,
        stripPrefix = strip_prefix,
    )

    # Provide all URLs to our mirror_to_s3 script.
    generate_repository_metadata(
        repo_ctx,
        repository_rule_type = "mosek",
        downloads = [
            {
                "urls": [
                    url.format(path = "stable/{}/{}".format(
                        version,
                        one_basename,
                    ))
                    for url in mirrors["mosek"]
                ],
                "sha256": one_checksum,
            }
            for one_basename, one_checksum in sha256.items()
        ],
    )

    # On macOS, adjust the MOSEK shared library to find itself via rpaths, as
    # required for Bazel's cc_import to work properly. The filename we want to
    # fix looks like "libmosek64.##.##.dylib".
    for item in repo_ctx.path("bin").readdir():
        name = item.basename
        if all([
            name.startswith("libmosek64"),
            name.endswith(".dylib"),
            name != "libmosek64.dylib",
        ]):
            execute_or_fail(repo_ctx, [
                "install_name_tool",
                "-id",
                "@rpath/" + name,
                "bin/" + name,
            ])

    # Add the BUILD file.
    repo_ctx.symlink(
        Label("@drake//tools/workspace/mosek:package.BUILD.bazel"),
        "BUILD.bazel",
    )

repository_impl = repository_rule(
    implementation = _impl,
    attrs = {
        "version": attr.string(
            mandatory = True,
        ),
        "mirrors": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
        "sha256": attr.string_dict(),
    },
)
