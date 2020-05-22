# -*- python -*-

load("@drake//tools/workspace:os.bzl", "determine_os")

def setup_pypi_wheel(
        repository_ctx,
        pypi_name = None,
        version = None,
        pypi_tag = None,
        blake2_256 = None,
        sha256 = None,
        pypi_wheel_mirrors = None,
        deps = [],
        imports = [],
        data = []):
    """XXX docs...
    """

    # Validate our args.
    pypi_name or fail("Missing required pypi_name")
    version or fail("Missing required version")
    pypi_tag or fail("Missing required pypi_tag")
    blake2_256 or fail("Missing required blake2_256")
    pypi_wheel_mirrors or fail("Missing required pypi_wheel_mirrors")
    sha256 = sha256 or "0" * 64

    # Download and extract the wheel.
    blake2_256_01 = blake2_256[:2]
    blake2_256_23 = blake2_256[2:4]
    blake2_256_4p = blake2_256[4:]
    urls = [
        pattern.format(
            name = pypi_name,
            version = version,
            tag = pypi_tag,
            blake2_256 = blake2_256,
            blake2_256_01 = blake2_256_01,
            blake2_256_23 = blake2_256_23,
            blake2_256_4p = blake2_256_4p,
            sha256 = sha256,
        )
        for pattern in pypi_wheel_mirrors
    ]
    repository_ctx.download_and_extract(
        url = urls,
        sha256 = sha256,
        type = "zip",
    )

    # Wheel files can store things at the top level in a usable way, or in a
    # separate data directory.  If we have the separate data directory, move
    # its contents to the repository root.
    data_dir = "{}-{}.data".format(pypi_name, version)
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
