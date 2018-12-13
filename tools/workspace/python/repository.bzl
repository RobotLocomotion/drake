# -*- mode: python -*-
# vi: set ft=python :

"""
Finds local system Python headers and libraries using python-config and
makes them available to be used as a C/C++ dependency. On macOS, Python
libraries should not typically be directly linked, so the :python target passes
the "-undefined dynamic_lookup" linker flag, however in the rare cases that
this would cause an undefined symbol error, a :python_direct_link target is
provided. On Linux, these targets are identical.

The Python distribution is determined by `--action_env=PYTHON_BIN_PATH=<bin>`,
which should match Bazel's version (via `--python_path=<bin>`).

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
"""

load("@drake//tools/workspace:execute.bzl", "execute_or_fail", "which")
load("@drake//tools/workspace:os.bzl", "determine_os")

_VERSION_SUPPORT_MATRIX = {
    "ubuntu:16.04": ["2.7"],
    "ubuntu:18.04": ["2.7"],
    "macos:10.13": ["2.7"],
    "macos:10.14": ["2.7"],
}

def _repository_python_info(repository_ctx):
    # Using `PYTHON_BIN_PATH` from the environment, determine:
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
        os_key = os_result.distribution + ":" + os_result.macos_release
    else:
        os_key = os_result.distribution + ":" + os_result.ubuntu_release
    versions_supported = _VERSION_SUPPORT_MATRIX[os_key]

    # Bazel does not easily expose its --python_path to repository rules (e.g.
    # the environment is unaffected). We must use a workaround as Tensorflow
    # does in `python_configure.bzl` (https://git.io/fx4Pp).
    # N.B. Unfortunately, it does not seem possible to get Bazel's Python
    # interpreter during a repository rule, thus we can only catch mismatch
    # issues via `//tools/workspace/python:py/python_bin_test`.
    if os_result.is_macos:
        version_supported_major, _ = versions_supported[0].split(".")

        # N.B. On Mac, `which python{major}.{minor}` may refer to the system
        # Python, not Homebrew Python.
        python_default = "python{}".format(version_supported_major)
    else:
        python_default = "python{}".format(versions_supported[0])
    python_from_env = repository_ctx.os.environ.get(
        "PYTHON_BIN_PATH",
        python_default,
    )
    python = str(which(repository_ctx, python_from_env))
    version = execute_or_fail(
        repository_ctx,
        [python, "-c", "from sys import version_info as v; print(\"{}.{}\"" +
                       ".format(v.major, v.minor))"],
    ).stdout.strip()
    version_major, _ = version.split(".")

    # Development Note: This should generally be the correct configuration. If
    # you are hacking with `virtualenv` (which is officially unsupported),
    # ensure that you manually symlink the matching `*-config` binary in your
    # `virtualenv` installation.
    python_config = "{}-config".format(python)

    # Warn if we do not the correct platform support.
    if version not in versions_supported:
        print((
            "\n\nWARNING: Python {} is not a supported / tested version for " +
            "use with Drake.\n  Supported versions on {}: {}\n\n"
        ).format(version, os_key, versions_supported))

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
    py_info = _repository_python_info(
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
        if not linkopts[i].startswith("-"):
            linkopts[i - 1] += " " + linkopts.pop(i)

    linkopts_direct_link = list(linkopts)

    if py_info.os.is_macos:
        for i in reversed(range(len(linkopts))):
            if linkopts[i].find("python" + py_info.version_major) != -1:
                linkopts.pop(i)
        linkopts = ["-undefined dynamic_lookup"] + linkopts

    skylark_content = """
# DO NOT EDIT: generated by python_repository()
# WARNING: Avoid using this macro in any repository rules which require
# `load()` at the WORKSPACE level. Instead, load these constants through
# `BUILD.bazel` or `package.BUILD.bazel` files.

PYTHON_BIN_PATH = "{bin_path}"
PYTHON_VERSION = "{version}"
PYTHON_SITE_PACKAGES_RELPATH = "{site_packages_relpath}"
""".format(
        bin_path = py_info.python,
        version = py_info.version,
        site_packages_relpath = py_info.site_packages_relpath,
    )
    repository_ctx.file(
        "version.bzl",
        content = skylark_content,
        executable = False,
    )
    repository_ctx.file(
        "bazel_python_actionenv.py",
        content = skylark_content,
        executable = False,
    )

    build_content = """# -*- python -*-

# DO NOT EDIT: generated by python_repository()

licenses(["notice"])  # Python-2.0

# Only include first level of headers included from `python_repository`
# (`include/<destination>/*`). This should exclude third party C headers which
# may be nested within `/usr/include/python<version>`, such as `numpy` when
# installed via `apt` on Ubuntu.
headers = glob(
    ["include/*/*"],
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
    deps = [":python_headers"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "python_direct_link",
    linkopts = {},
    deps = [":python_headers"],
    visibility = ["//visibility:public"],
)

py_library(
    name = "bazel_python_actionenv",
    srcs = ["bazel_python_actionenv.py"],
    imports = ["."],
    visibility = ["//visibility:public"],
    testonly = 1,
)
    """.format(includes, linkopts, linkopts_direct_link)

    repository_ctx.file(
        "BUILD.bazel",
        content = build_content,
        executable = False,
    )

python_repository = repository_rule(
    _impl,
    environ = [
        "PYTHON_BIN_PATH",
    ],
    local = True,
)
