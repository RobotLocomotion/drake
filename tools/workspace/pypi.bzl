# -*- mode: python -*-
# vi: set ft=python :

load(
    "@bazel_tools//tools/build_defs/repo:utils.bzl",
    "workspace_and_buildfile",
)
load("@drake//tools/workspace:metadata.bzl", "generate_repository_metadata")

def pypi_archive(
        name,
        package = None,
        version = None,
        sha256 = None,
        strip_prefix = "",
        build_file = None,
        build_file_content = None,
        workspace_file = None,
        workspace_file_content = None,
        mirrors = None):
    """Downloads and unpacks a PyPI package archive and adds it to the
    WORKSPACE as an external.

    Example:
        Download and use the "foo" package, version 1.2.3, hosted on PyPI at
        https://files.pythonhosted.org/packages/source/f/foo/foo-1.2.3.tar.gz.

        WORKSPACE:
            load("//tools/workspace:pypi.bzl", "pypi_archive")
            pypi_archive(
                name = "foo",
                version = "1.2.3",
                build_file = "foo.BUILD",
                sha256 = "0123456789abcdef...",
            )

        foo.BUILD:
            load("//tools/skylark:py.bzl", "py_library")
            py_library(
                name = "foo",
                srcs = [
                    "foo/__init__.py",
                    "foo/bar.py",
                ],
                visibility = ["//visibility:public"],
            )

        BUILD:
            load("//tools/skylark:py.bzl", "py_binary")
            py_binary(
                name = "foobar",
                deps = ["@foo//:foo"],
                srcs = ["foobar.py"],
            )

    Arguments:
        name: A unique name for this rule. This argument will be used for the
            package name if the "package" argument is omitted [Name; required].

        package: The name of the PyPI package to download. The "name" argument
            will be used if this argument is omitted [String; optional].

        version: The version of the PyPI package to be downloaded
            [String; required].

        sha256: The expected SHA-256 hash of the archive to download. This
            argument must match the SHA-256 hash of the downloaded archive.
            The download will fail if omitted, but the checksum-mismatch error
            message will offer a suggestion for the correct value of this
            argument [String; required].

        strip_prefix: A directory prefix to strip from the extracted files
            [String; optional].

        build_file: The file to use as the BUILD file for this repository.
            This argument is an absolute label. Either build_file or
            build_file_content must be specified, but not both
            [Label; optional].

        build_file_content: The content for the BUILD file for this repository.
            Either build_file or build_file_content must be specified, but not
            both [Label; optional].

        workspace_file: The file to use as the WORKSPACE file for this
            repository. This argument is an absolute label. Either
            workspace_file or workspace_file_content may be specified, or
            neither, but not both [Label; optional].

        workspace_file_content: The content for the WORKSPACE file for this
            repository. Either workspace_file, workspace_file_content or
            neither may be specified, but not both [Label; optional].

        mirrors: A dict from string to list-of-string with key "pypi", where
            the list-of-strings are URLs to use, formatted using {package},
            {version}, and {p} (where {p} is the first letter of {package}).
    """
    if not package:
        package = name

    if not version:
        fail("The version argument to pypi_archive is required.")

    if not build_file and not build_file_content:
        fail("Either the build_file or build_file_content argument to " +
             "pypi_archive is required.")

    _pypi_archive(
        name = name,
        package = package,
        version = version,
        sha256 = sha256,
        strip_prefix = strip_prefix,
        build_file = build_file,
        build_file_content = build_file_content,
        workspace_file = workspace_file,
        workspace_file_content = workspace_file_content,
        mirrors = mirrors,
    )

def setup_pypi_repository(repository_ctx):
    """Downloads and unpacks a PyPI package archive and adds it to the
    WORKSPACE as an external.

    Args:
        repository_ctx: context of a Bazel repository rule.
    """
    pypi_download_and_extract(
        repository_ctx,
        package = repository_ctx.attr.package,
        version = repository_ctx.attr.version,
        mirrors = repository_ctx.attr.mirrors,
        sha256 = repository_ctx.attr.sha256,
        strip_prefix = repository_ctx.attr.strip_prefix,
    )

    workspace_and_buildfile(repository_ctx)

    return struct(error = None)

def pypi_download_and_extract(
        repository_ctx,
        package,
        version,
        mirrors,
        output = "",
        sha256 = None,
        strip_prefix = ""):
    """Downloads an archive of the provided PyPI package and version to the
    output path and extracts it.

    Args:
        repository_ctx: context of a Bazel repository rule.
        package: PyPI package name.
        version: version for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
        output: path to the directory where the archive will be unpacked,
            relative to the Bazel repository directory.
        sha256: expected SHA-256 hash of the archive downloaded. Fallback to
            an incorrect default value to prevent the hash check from being
            disabled, but allow the first download attempt to fail and print
            the correct SHA-256 hash.
        strip_prefix: additional directory prefix to strip from the extracted
            files.
    """
    urls = _urls(package, version, mirrors)

    repository_ctx.download_and_extract(
        urls,
        output = output,
        sha256 = _sha256(sha256),
        type = "tar.gz",
        stripPrefix = _strip_prefix(package, version, strip_prefix),
    )

    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "pypi",
        package = package,
        version = version,
        sha256 = sha256,
        urls = urls,
    )

def _sha256(sha256):
    """Fallback to an incorrect default value of SHA-256 hash to prevent the
    hash check from being disabled, but allow the first download attempt of an
    archive to fail and print the correct hash.

    Args:
        sha256: expected SHA-256 hash of the archive to be downloaded.
    """
    if not sha256:
        sha256 = "0" * 64

    return sha256

def _strip_prefix(package, version, strip_prefix = ""):
    """Computes the strip prefix for a downloaded archive of the provided
    PyPI package and version.

    Args:
        package: PyPI package name.
        version: version for which the archive should be downloaded.
        strip_prefix: additional directory prefix to strip from the extracted
            files.
    """
    if strip_prefix:
        return "{0}-{1}/{2}".format(package, version, strip_prefix)

    return "{0}-{1}".format(package, version)

def _urls(package, version, mirrors):
    """Computes the urls from which an archive of the provided PyPI package and
    version should be downloaded.

     Args:
        package: PyPI package name.
        version: version for which the archive should be downloaded.
        mirrors: dictionary of mirrors, see mirrors.bzl in this directory for
            an example.
    """
    return [
        x.format(
            p = package[:1],
            package = package,
            version = version,
        )
        for x in mirrors.get("pypi")
    ]

def _pypi_archive_impl(repository_ctx):
    result = setup_pypi_repository(repository_ctx)

    if result.error:
        fail("Unable to complete setup for @{} repository: {}".format(
            repository_ctx.name,
            result.error,
        ))

_pypi_archive = repository_rule(
    attrs = {
        "package": attr.string(),
        "version": attr.string(),
        "sha256": attr.string(),
        "strip_prefix": attr.string(),
        "build_file": attr.label(allow_single_file = True),
        "build_file_content": attr.string(),
        "workspace_file": attr.label(),
        "workspace_file_content": attr.string(),
        "mirrors": attr.string_list_dict(
            mandatory = True,
            allow_empty = False,
        ),
    },
    environ = ["BAZEL_SH"],
    implementation = _pypi_archive_impl,
)
