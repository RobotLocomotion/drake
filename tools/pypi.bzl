# -*- mode: python -*-
# vi: set ft=python :

def pypi_archive(
        name,
        package = None,
        version = None,
        build_file = None,
        sha256 = None,
        strip_prefix = None,
        **kwargs):
    """
    Downloads and unpacks a PyPI package archive and adds it to the WORKSPACE
    as an external. Additional keyword arguments (except "urls") will be passed
    through to the native call of "new_http_archive."

    Example:
        Download and use the "foo" package, version 1.2.3, hosted on PyPI at
        https://files.pythonhosted.org/packages/source/f/foo/foo-1.2.3.tar.gz.

        WORKSPACE:
            load("//tools:pypi_archive.bzl", "pypi_archive")
            pypi_archive(
                name = "foo",
                version = "1.2.3",
                build_file = "foo.BUILD",
                sha256 = "0123456789abcdef...",
            )

        foo.BUILD:
            py_library(
                name = "foo",
                srcs = [
                    "foo/__init__.py",
                    "foo/bar.py",
                ],
                visibility = ["//visibility:public"],
            )

        BUILD:
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

        build_file: The file to use as the BUILD file for this repository.
            This argument is a label relative to the WORKSPACE
            [String; required].

        sha256: The expected SHA-256 hash of the archive to download. This
            argument must match the SHA-256 hash of the downloaded archive.
            The download will fail if omitted, but the checksum-mismatch error
            message will offer a suggestion for the correct value of this
            argument [String; required].

        strip_prefix: A directory prefix to strip from the extracted files
            [String; optional].
    """
    if not package:
        package = name

    if not version:
        fail("The version argument to pypi_archive is required.")

    if not build_file:
        fail("The build_file argument to pypi_archive is required.")

    if not sha256:
        # Set an incorrect default value to allow the download attempt to fail
        # and print a suggested SHA-256 checksum in the checksum-mismatch error
        # message.
        sha256 = "0" * 64

    if strip_prefix:
        strip_prefix = "{0}-{1}/{2}".format(package, version, strip_prefix)
    else:
        strip_prefix = "{0}-{1}".format(package, version)

    # Packages are mirrored from PyPI to CloudFront backed by an S3 bucket.
    urls = [
        "https://files.pythonhosted.org/packages/source/{0}/{1}/{1}-{2}.tar.gz".format(  # noqa
            package[:1], package, version),
        "https://d2tbce6hkathzp.cloudfront.net/pypi/{0}/{0}-{1}.tar.gz".format(
            package, version),
        "https://s3.amazonaws.com/drake-mirror/pypi/{0}/{0}-{1}.tar.gz".format(
            package, version),
    ]

    native.new_http_archive(
        name = name,
        build_file = build_file,
        sha256 = sha256,
        strip_prefix = strip_prefix,
        urls = urls,
        **kwargs)
