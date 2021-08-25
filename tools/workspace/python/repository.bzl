# -*- mode: python -*-
# vi: set ft=python :

"""
Finds local system Python headers and libraries using python-config and
makes them available to be used as a C/C++ dependency. On macOS, Python
libraries should not typically be directly linked, so the :python target passes
the "-undefined dynamic_lookup" linker flag, however in the rare cases that
this would cause an undefined symbol error, a :python_direct_link target is
provided. On Linux, these targets are identical.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/python:repository.bzl", "python_repository")  # noqa
        python_repository(
            name = "foo",
        )

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:python"],
            srcs = ["bar.cc"],
        )

Arguments:
    name: A unique name for this rule.
    linux_interpreter_path: Optional interpreter path for the Python runtime in
        the registered Python toolchain on the @platforms//os:linux platform.
        Defaults to the value of LINUX_INTERPRETER_PATH in
        //tools/py_toolchain:interpreter_paths.bzl.
    macos_interpreter_path: Optional interpreter path for the Python runtime in
        the registered Python toolchain on the @platforms//os:osx (macOS)
        platform. Defaults to the value of MACOS_INTERPRETER_PATH in
        //tools/py_toolchain:interpreter_paths.bzl.
"""

load(
    "@drake//tools/py_toolchain:interpreter_paths.bzl",
    "LINUX_INTERPRETER_PATH",
    "MACOS_INTERPRETER_PATH",
)
load("@drake//tools/workspace:execute.bzl", "execute_or_fail", "which")
load("@drake//tools/workspace:os.bzl", "determine_os")

# The supported Python versions should match those listed in both the root
# CMakeLists.txt and doc/developers.rst.
_VERSION_SUPPORT_MATRIX = {
    "ubuntu:18.04": ["3.6"],
    "ubuntu:20.04": ["3.8"],
    "macos": ["3.9"],
    "manylinux": ["3.6"],
}

def repository_python_info(repository_ctx):
    # Given the operating system, determine:
    # - `python` - binary path
    # - `python_config` - configuration binary path
    # - `site_packages_relpath` - relative to base of FHS
    # - `version` - '{major}.{minor}`
    # - `version_major` - major version
    # - `os` - results from `determine_os(...)`
    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)
    if os_result.is_macos:
        os_key = os_result.distribution
    elif os_result.is_ubuntu:
        os_key = os_result.distribution + ":" + os_result.ubuntu_release
    else:
        os_key = "manylinux"
    versions_supported = _VERSION_SUPPORT_MATRIX[os_key]

    if os_result.is_macos:
        # This value must match the interpreter_path in
        # @drake//tools/py_toolchain:macos_py3_runtime
        python = repository_ctx.attr.macos_interpreter_path
    else:
        # This value must match the interpreter_path in
        # @drake//tools/py_toolchain:linux_py3_runtime
        python = repository_ctx.attr.linux_interpreter_path

    version = execute_or_fail(
        repository_ctx,
        [python, "-c", "from sys import version_info as v; print(\"{}.{}\"" +
                       ".format(v.major, v.minor))"],
    ).stdout.strip()
    version_major, _ = version.split(".")

    # Perform sanity checks on supplied Python binary.
    implementation = execute_or_fail(
        repository_ctx,
        [
            python,
            "-c",
            "import platform as m; print(m.python_implementation())",
        ],
    ).stdout.strip()
    if implementation != "CPython":
        fail(("The implementation of '{}' is '{}', but only 'CPython' is " +
              "supported.").format(python, implementation))

    # Development Note: This should generally be the correct configuration. If
    # you are hacking with `virtualenv` (which is officially unsupported),
    # ensure that you manually symlink the matching `*-config` binary in your
    # `virtualenv` installation.
    python_config = "{}-config".format(python)

    # Check if config binary exists.
    if which(repository_ctx, python_config) == None:
        fail((
            "Cannot find corresponding config executable: {}\n" +
            "  From interpreter: {}"
        ).format(python_config, python))

    # Warn if we do not the correct platform support.
    if version not in versions_supported:
        print((
            "\n\nWARNING: Python {} is not a supported / tested version for " +
            "use with Drake.\n  Supported versions on {}: {}\n  " +
            "From interpreter: {}\n\n"
        ).format(version, os_key, versions_supported, python))

    site_packages_relpath = "lib/python{}/site-packages".format(version)
    return struct(
        python = python,
        python_config = python_config,
        site_packages_relpath = site_packages_relpath,
        version = version,
        version_major = version_major,
        os = os_result,
    )

def _impl(repository_ctx):
    # Repository implementation.
    py_info = repository_python_info(
        repository_ctx,
    )

    # Collect includes.
    cflags = execute_or_fail(
        repository_ctx,
        [py_info.python_config, "--includes"],
    ).stdout.strip().split(" ")
    cflags = [cflag for cflag in cflags if cflag]

    root = repository_ctx.path("")
    root_len = len(str(root)) + 1
    base = root.get_child("include")

    # TODO(jamiesnape): Much of the logic for parsing flags is the same or
    # similar to that used in pkg_config.bzl and should be refactored and
    # shared instead of being duplicated in both places.

    extension_suffix = execute_or_fail(
        repository_ctx,
        [py_info.python_config, "--extension-suffix"],
    ).stdout.strip()

    includes = []

    for cflag in cflags:
        if cflag.startswith("-I"):
            source = repository_ctx.path(cflag[2:])
            destination = base.get_child(str(source).replace("/", "_"))
            include = str(destination)[root_len:]

            if include not in includes:
                repository_ctx.symlink(source, destination)
                includes += [include]

    # Collect linker paths.
    linkopts = execute_or_fail(
        repository_ctx,
        [py_info.python_config, "--ldflags"],
    ).stdout.strip().split(" ")
    linkopts = [linkopt for linkopt in linkopts if linkopt]

    for i in reversed(range(len(linkopts))):
        link_prefix = "-L"
        if linkopts[i].startswith(link_prefix):
            linkopts.insert(i, "-Wl,-rpath," + linkopts[i][len(link_prefix):])
        if not linkopts[i].startswith("-"):
            linkopts[i - 1] += " " + linkopts.pop(i)

    linkopts_direct_link = list(linkopts)

    # python3.9-config --libs is missing the python3.9 library.
    has_direct_link = False
    libpy = "python" + py_info.version
    for i in reversed(range(len(linkopts))):
        if linkopts[i].startswith("-l") and linkopts[i].find(libpy) != -1:
            has_direct_link = True
            if py_info.os.is_macos:
                linkopts.pop(i)

    if py_info.os.is_macos:
        linkopts = ["-undefined dynamic_lookup"] + linkopts

    if not has_direct_link:
        linkopts_direct_link = ["-l" + libpy] + linkopts_direct_link

    skylark_content = """
# DO NOT EDIT: generated by python_repository()
# WARNING: Avoid using this macro in any repository rules which require
# `load()` at the WORKSPACE level. Instead, load these constants through
# `BUILD.bazel` or `package.BUILD.bazel` files.

PYTHON_BIN_PATH = "{bin_path}"
PYTHON_EXTENSION_SUFFIX = "{extension_suffix}"
PYTHON_VERSION = "{version}"
PYTHON_SITE_PACKAGES_RELPATH = "{site_packages_relpath}"
""".format(
        bin_path = py_info.python,
        extension_suffix = extension_suffix,
        version = py_info.version,
        site_packages_relpath = py_info.site_packages_relpath,
    )
    repository_ctx.file(
        "version.bzl",
        content = skylark_content,
        executable = False,
    )

    build_content = """# -*- python -*-

# DO NOT EDIT: generated by python_repository()

licenses(["notice"])  # Python-2.0

# Only include the first level of headers and specific second level headers
# included from `python_repository`. This excludes some third-party C headers
# that may be nested within `/usr/include/python<version>`, such as `numpy`,
# when installed via `apt` on Ubuntu.
headers = glob(
    [
        "include/*/*",
        "include/*/cpython/*",
        "include/*/internal/*",
    ],
    exclude_directories = 1,
)

cc_library(
    name = "python_headers",
    hdrs = headers,
    includes = {},
    visibility = ["//visibility:private"],
)

cc_library(
    name = "python",
    linkopts = {},
    visibility = ["//visibility:public"],
    deps = [":python_headers"],
)

cc_library(
    name = "python_direct_link",
    linkopts = {},
    visibility = ["//visibility:public"],
    deps = [":python_headers"],
)
""".format(includes, linkopts, linkopts_direct_link)

    repository_ctx.file(
        "BUILD.bazel",
        content = build_content,
        executable = False,
    )

interpreter_path_attrs = {
    # The value of this argument should match the interpreter_path for
    # the py_runtime in the registered Python toolchain on the
    # @platforms//os:linux platform.
    "linux_interpreter_path": attr.string(default = LINUX_INTERPRETER_PATH),
    # The value of this argument should match the interpreter_path for
    # the py_runtime in the registered Python toolchain on the
    # @platforms//os:osx platform.
    "macos_interpreter_path": attr.string(default = MACOS_INTERPRETER_PATH),
}

python_repository = repository_rule(
    _impl,
    attrs = interpreter_path_attrs,
    local = True,
    configure = True,
)
