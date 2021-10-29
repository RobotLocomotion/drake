# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")
load("@drake//tools/workspace:metadata.bzl", "generate_repository_metadata")

def download_and_extract_pypi_wheel(
        repository_ctx,
        package = None,
        version = None,
        version_pin = None,
        pypi_tag = None,
        blake2_256 = None,
        sha256 = None,
        mirrors = None):
    """Downloads and unpacks a PyPI wheel.

    Arguments:
        package: The name of the PyPI package to download [String; required].

        version: The version of the PyPI package to download
            [String; required].

        version_pin: True iff the wheel should remain at the same version
            indefinitely, eschewing automated upgrades to newer versions.
            [Bool; optional].

        pypi_tag: The tag of the PyPI package to download, e.g.,
            "cp36-cp36m-manylinux1_x86_64".
            This is seen on the PyPI webpage under "Download Files" then "View"
            [String; required].

        blake2_256: The expected BLAKE2-256 hash of the archive to download.
            This is seen on the PyPI webpage under "Download Files" then "View"
            [String; required].

        sha256: The expected SHA-256 hash of the archive to download. This
            argument must match the SHA-256 hash of the downloaded archive.
            The download will fail if omitted, but the checksum-mismatch error
            message will offer a suggestion for the correct value of this
            argument [String; required].

        mirrors: A dict from string to list-of-string with key "pypi_wheel",
            where the list-of-strings are URLs to use, formatted using
            {package}, {version}, {tag}, {sha256}, {blake2_256},
            {blake2_256_01}, {blake2_256_23}, {blake2_256_4p} where the final
            three are respectively the first two characters of {blake2_256},
            second two characters of {blake2_256}, and remaining characters
            of {blake2_256}.
    """

    # Validate our args.
    package or fail("Missing required package")
    version or fail("Missing required version")
    pypi_tag or fail("Missing required pypi_tag")
    blake2_256 or fail("Missing required blake2_256")
    mirrors or fail("Missing required mirrors")
    sha256 = sha256 or "0" * 64

    # Download and extract the wheel.
    urls = [
        pattern.format(
            package = package,
            version = version,
            tag = pypi_tag,
            blake2_256 = blake2_256,
            blake2_256_01 = blake2_256[:2],
            blake2_256_23 = blake2_256[2:4],
            blake2_256_4p = blake2_256[4:],
            sha256 = sha256,
        )
        for pattern in mirrors["pypi_wheel"]
    ]
    repository_ctx.download_and_extract(
        url = urls,
        sha256 = sha256,
        type = "zip",
    )

    # Wheel files can store things at the top level in a usable way, or in a
    # separate data directory.  If we have the separate data directory, move
    # its contents to the repository root.
    data_dir = "{}-{}.data".format(package, version)
    if repository_ctx.path(data_dir).exists:
        data_subdirs = repository_ctx.path(data_dir).readdir()
        for data_subdir in data_subdirs:
            for to_move in data_subdir.readdir():
                result = repository_ctx.execute(["mv", to_move, "."])
                if result.return_code:
                    fail("{} error moving file {}".format(
                        repository_ctx.name,
                        to_move,
                    ))

    # Tell our mirroring scripts what we downloaded.
    generate_repository_metadata(
        repository_ctx,
        repository_rule_type = "pypi_wheel",
        package = package,
        version = version,
        version_pin = version_pin,
        pypi_tag = pypi_tag,
        blake2_256 = blake2_256,
        sha256 = sha256,
        urls = urls,
    )

    return struct(error = None)

def setup_pypi_wheel(
        repository_ctx,
        package = None,
        version = None,
        version_pin = None,
        pypi_tag = None,
        blake2_256 = None,
        sha256 = None,
        mirrors = None,
        deps = [],
        imports = [],
        data = []):
    """Downloads and unpacks a PyPI wheel and adds it to the WORKSPACE as an
    external.

    Arguments:
        package, version, pypi_tag, blake2_256, sha256, mirrors: Arguments to
            be passed to download_and_extract_pypi_wheel() [Required].

        version_pin: Arguments to be passed to
            download_and_extract_pypi_wheel() [Optional].

        deps, imports, data: Additional attrs for the target in the BUILD file
            [Optional].
    """

    # Download and extract the wheel.
    download_and_extract_pypi_wheel(
        repository_ctx,
        package = package,
        version = version,
        version_pin = version_pin,
        pypi_tag = pypi_tag,
        blake2_256 = blake2_256,
        sha256 = sha256,
        mirrors = mirrors,
    )

    # Create the BUILD file.
    BUILD = """
load("@drake//tools/skylark:py.bzl", "py_library")

py_library(
    name = "{name}",
    srcs = glob(["**/*.py"]),
    data = glob(["**/*.so", "**/*.so.*"]) + {extra_data},
    deps = {deps},
    imports = {imports},
    visibility = ["//visibility:public"],
)
    """.format(
        name = repository_ctx.name,
        extra_data = data,
        deps = [str(x) for x in deps],
        imports = imports,
    )
    repository_ctx.file("BUILD.bazel", BUILD)

    return struct(error = None)
