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
load("//tools/workspace:execute.bzl", "execute_or_fail")

def _check_compiler_version(compiler_id, actual_version, supported_version):
    """
    Check if the compiler is of a supported version and report an error if not.

    Both actual_version and supported_version must be a 2-tuple of the
    (major, minor) version.
    """
    if actual_version[0] > supported_version[0]:
        pass

    elif (actual_version[0] == supported_version[0] and
          actual_version[1] >= supported_version[1]):
        pass

    else:
        fail("{} compiler version {}.{} is less than {}.{}.".format(
            compiler_id,
            actual_version[0],
            actual_version[1],
            supported_version[0],
            supported_version[1] if len(supported_version) > 1 else 0,
        ))

def _impl(repository_ctx):
    fail("Disable CI")
    file_content = """# DO NOT EDIT: generated by cc_repository()

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
    compiler_version_major = int(output[1])
    compiler_version_minor = int(output[2])

    # The minimum compiler versions should match those listed in both the root
    # CMakeLists.txt and doc/_pages/from_source.md.

    if repository_ctx.os.name == "mac os x":
        supported_compilers = {"AppleClang": (14, 0)}
    else:
        supported_compilers = {"Clang": (12, 0), "GNU": (9, 3)}

    if compiler_id in supported_compilers:
        _check_compiler_version(
            compiler_id,
            (compiler_version_major, compiler_version_minor),
            supported_compilers[compiler_id],
        )
    else:
        # We do not fail outright here since even though we do not officially
        # support them, Drake may happily compile with new enough versions of
        # compilers that are compatible with GNU flags such as -std=c++17.

        print("WARNING: {} is NOT a supported C/C++ compiler.".format(
            compiler_id,
        ))
        print("WARNING: Compilation of the drake WORKSPACE may fail.")

    file_content = """# DO NOT EDIT: generated by cc_repository()

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
