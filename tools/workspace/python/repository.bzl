# -*- mode: python -*-
# vi: set ft=python :

"""
Finds local system Python headers and libraries using python-config and
makes them available to be used as a C/C++ dependency. On macOS, Python
libraries should not typically be directly linked, so the :python target passes
the "-undefined dynamic_lookup" linker flag, however in the rare cases that
this would cause an undefined symbol error, a :python_direct_link target is
provided. On Linux, these targets are identical.

The Python distribution is determined by
`--action_env=DRAKE_PYTHON_BIN_PATH=<bin>`, which should match Bazel's version
(via `--python_path=<bin>`).

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

# The supported Python versions should match those listed in both the root
# CMakeLists.txt and doc/developers.rst.
_VERSION_SUPPORT_MATRIX = {
    "ubuntu:16.04": ["2.7"],
    "ubuntu:18.04": ["2.7", "3.6"],
    "macos:10.13": ["2.7", "3.7"],
    "macos:10.14": ["2.7", "3.7"],
}

def _repository_python_info(repository_ctx):
    # Using `DRAKE_PYTHON_BIN_PATH` from the environment, determine:
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

    # Bazel does not easily expose its --python_path to repository rules
    # (analysis phase). We must use a workaround as Tensorflow does in
    # `python_configure.bzl` (https://git.io/fx4Pp). We check for consistency
    # during the build (execution) phase using `bazel_python_is_valid`.
    python_path = repository_ctx.os.environ.get("DRAKE_PYTHON_BIN_PATH")
    if python_path == None:
        # TODO(eric.cousineau): Make this an error once `.bazelrc` stops using
        # `try-import` for configuration.
        if os_result.is_macos:
            python_path = "/usr/local/bin/python{}".format(
                versions_supported[0],
            )
        else:
            python_path = "/usr/bin/python{}".format(versions_supported[0])
    if not python_path.startswith("/"):
        fail("`--action_env=DRAKE_PYTHON_BIN_PATH` must provide an " +
             "absolute path.")
    if which(repository_ctx, python_path) == None:
        fail("Executable does not exist: {}".format(python_path))
    python = str(python_path)
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
            "  For interpreter: {}"
        ).format(python_config, python_path))

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
    repository_ctx.symlink(
        Label("@drake//tools/workspace/python:check_bazel_python.py"),
        "_check_bazel_python.py",
    )
    repository_ctx.file(
        "_bazel_python_actionenv.py",
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
    # Depend on a Python configuration sanity check for anything that wishes to
    # generate bindings. See `genrule` below for more information.
    data = [":bazel_python_is_valid"],
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

# See `genrule` below.
py_binary(
    name = "check_bazel_python",
    main = "_check_bazel_python.py",
    srcs = [
        "_check_bazel_python.py",
        "_bazel_python_actionenv.py",
    ],
    imports = ["."],
    visibility = ["//visibility:private"],
)

# Place this test as a `genrule` to (a) test at build time and (b) be able to
# access Bazel's Python interpreter from a `py_binary` used in `tools`.
genrule(
    name = "bazel_python_is_valid",
    outs = [".bazel_python_is_valid"],
    cmd = "$(location :check_bazel_python) > $@",
    tools = [":check_bazel_python"],
    visibility = ["//visibility:private"],
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
        "DRAKE_PYTHON_BIN_PATH",
    ],
    local = True,
)
