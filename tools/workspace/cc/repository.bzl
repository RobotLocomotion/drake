# -*- mode: python -*-
# vi: set ft=python :

"""
Identifies the C/C++ compiler by examining the presence or values of various
predefined C preprocessor macros. Identifies any compiler capable of compiling
C++ code that is supported by CMake 3.12.0.

Note that there are constraint_values @bazel_tools//tools/cpp:clang and
@bazel_tools//tools/cpp:gcc that could potentially distinguish between the
Clang and GCC compilers as an alternative to this approach, but as of Bazel
0.14.1, they appear not to be compatible with the autogenerated toolchain.

Example:
        load("@drake//tools/workspace/cc:repository.bzl", "cc_repository")
        cc_repository(name = "cc")

    foo.bzl:
        load("@cc//:compiler.bzl", "COMPILER_ID")

        if "COMPILER_ID" == "AppleClang":
            # Do something...

        if "COMPILER_ID" == "Clang":
            # Do something...

        if "COMPILER_ID" == "GNU":
            # Do something...

Argument:
    name: A unique name for this rule.
"""

load("@bazel_tools//tools/cpp:unix_cc_configure.bzl", "find_cc")
load("@drake//tools/workspace:execute.bzl", "execute_or_fail")

def _impl(repository_ctx):
    file_content = """# -*- python -*-

# DO NOT EDIT: generated by cc_repository()

# This file exists to make our directory into a Bazel package, so that our
# neighboring *.bzl file can be loaded elsewhere.
"""

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

    # https://github.com/bazelbuild/bazel/blob/1.1.0/tools/cpp/cc_configure.bzl
    if repository_ctx.os.environ.get("BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN", "0") == "1":  # noqa
        fail("Could NOT identify C/C++ compiler because CROSSTOOL is empty.")

    if repository_ctx.os.name == "mac os x" and repository_ctx.os.environ.get("BAZEL_USE_CPP_ONLY_TOOLCHAIN", "0") != "1":  # noqa
        # https://github.com/bazelbuild/bazel/blob/1.1.0/tools/cpp/osx_cc_configure.bzl
        cc = repository_ctx.path(Label("@local_config_cc//:wrapped_clang"))

        result = execute_or_fail(repository_ctx, [
            "xcode-select",
            "--print-path",
        ])
        developer_dir = result.stdout.strip()

        result = execute_or_fail(repository_ctx, [
            "xcrun",
            "--show-sdk-path",
        ])
        sdkroot = result.stdout.strip()

        cc_environment = {
            "DEVELOPER_DIR": developer_dir,
            "SDKROOT": sdkroot,
        }

    else:
        # https://github.com/bazelbuild/bazel/blob/1.1.0/tools/cpp/unix_cc_configure.bzl
        cc = find_cc(repository_ctx, overriden_tools = {})
        cc_environment = {}

    executable = repository_ctx.path("identify_compiler")
    execute_or_fail(repository_ctx, [
        cc,
        repository_ctx.path(
            Label("@drake//tools/workspace/cc:identify_compiler.cc"),
        ),
        "-o",
        executable,
    ], environment = cc_environment)

    result = execute_or_fail(repository_ctx, [executable])
    output = result.stdout.strip().split(" ")
    if len(output) != 3:
        fail("Could NOT identify C/C++ compiler.")

    compiler_id = output[0]

    if repository_ctx.os.name == "mac os x":
        supported_compilers = ["AppleClang"]
    else:
        supported_compilers = ["Clang", "GNU"]

    # We do not fail outright here since even though we do not officially
    # support them, Drake may happily compile with new enough versions of
    # compilers that are compatible with GNU flags such as -std=c++17.

    if compiler_id not in supported_compilers:
        print("WARNING: {} is NOT a supported C/C++ compiler.".format(
            compiler_id,
        ))
        print("WARNING: Compilation of the drake WORKSPACE may fail.")

    compiler_version_major = int(output[1])
    compiler_version_minor = int(output[2])

    # The minimum compiler versions should match those listed in both the root
    # CMakeLists.txt and doc/developers.rst. We know from experience that
    # compilation of Drake will certainly fail with versions lower than these,
    # even if they happen to support the necessary compiler flags.

    if compiler_id == "AppleClang":
        if compiler_version_major < 11:
            fail("AppleClang compiler version {}.{} is less than 11.0.".format(
                compiler_version_major,
                compiler_version_minor,
            ))

    elif compiler_id == "Clang":
        if compiler_version_major < 9:
            fail("Clang compiler version {}.{} is less than 9.0".format(
                compiler_version_major,
                compiler_version_minor,
            ))

    elif compiler_id == "GNU":
        if compiler_version_major < 7 or (compiler_version_major == 7 and
                                          compiler_version_minor < 5):
            fail("GNU compiler version {}.{} is less than 7.5.".format(
                compiler_version_major,
                compiler_version_minor,
            ))

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by cc_repository()

COMPILER_ID = "{}"

COMPILER_VERSION_MAJOR = {}
COMPILER_VERSION_MINOR = {}

""".format(compiler_id, compiler_version_major, compiler_version_minor)

    repository_ctx.file(
        "compiler.bzl",
        content = file_content,
        executable = False,
    )

cc_repository = repository_rule(
    environ = [
        "BAZEL_DO_NOT_DETECT_CPP_TOOLCHAIN",
        "BAZEL_USE_CPP_ONLY_TOOLCHAIN",
        "CC",
    ],
    configure = True,
    implementation = _impl,
)
