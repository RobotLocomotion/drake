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
            version = "2",
        )

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:python"],
            srcs = ["bar.cc"],
        )

Arguments:
    name: A unique name for this rule.
    version: The major or major.minor version of Python headers and libraries
        to be found. If set to "bazel", it will use the interpreter that Bazel
        is using (which can be specified via `--python_path`).
"""

load("@drake//tools/workspace:execute.bzl", "which")
load("@drake//tools/workspace:os.bzl", "determine_os")

_VERSION_SUPPORT_MATRIX = {
    "ubuntu:16.04": ["2.7"],
    "ubuntu:18.04": ["2.7"],
    "macOS:10.13": ["2.7"],
    "macOS:10.14": ["2.7"],
}

def _which(repository_ctx, bin_name):
    bin = repository_ctx.which(bin_name)
    if not bin:
        fail("Could NOT find {}".format(bin_name))
    return struct(ctx = repository_ctx, bin = bin)

def _exec(exec_ctx, args, name = None):
    args = [exec_ctx.bin] + args
    result = exec_ctx.ctx.execute(args)
    if name == None:
        name = args
    if result.return_code != 0:
        fail("Could not execute {}: {}".format(name, result.stderr))
    return result.stdout.strip()

def _repository_python_info(repository_ctx, version_user):
    if version_user == "bazel":
        python_bazel = _which(repository_ctx, "python")
        version_user = _exec(
            python_bazel,
            ["-c", "import sys; print(sys.version_info.major)"],
        )
    python_config = _which(
        repository_ctx,
        "python{}-config".format(version_user),
    )
    python = _which(repository_ctx, "python{}".format(version_user))

    os_result = determine_os(repository_ctx)
    if os_result.error != None:
        fail(os_result.error)

    version = _exec(
        python,
        ["-c", "from sys import version_info as v; print(\"{}.{}\"" +
               ".format(v.major, v.minor))"],
    )
    version_major, _ = version.split(".")

    # Estimate that we're using the same configuration between
    # `python{version}` and `python-config{version}`.
    python_configdir = _exec(
        python,
        ["-c", "import sysconfig; print(sysconfig.get_config_var(\"LIBPL\"))"],
    )
    python_config_configdir = _exec(python_config, ["--configdir"])
    if python_configdir != python_config_configdir:
        fail("Mismatch in configdir:\n  {}: {}\n  {}: {}".format(
            python.bin,
            python_configdir,
            python_config.bin,
            python_config_configdir,
        ))

    # Ensure we have the correct platform support.
    if os_result.is_macos:
        os_key = os_result.distribution + ":" + os_result.macos_release
    else:
        os_key = os_result.distribution + ":" + os_result.ubuntu_release
    versions_supported = _VERSION_SUPPORT_MATRIX[os_key]
    if version not in versions_supported:
        msg = (
            "Python {} is not a supported / tested version for use with " +
            "Drake.\n  Supported versions: {}\n"
        ).format(version, versions_supported)
        fail(msg)

    site_packages_relpath = "lib/python{}/site-packages".format(version)
    return python, python_config, struct(
        site_packages_relpath = site_packages_relpath,
        version = version,
        version_major = version,
        os = os_result,
    )

def _impl(repository_ctx):
    # Repository implementation.
    python, python_config, py_info = _repository_python_info(
        repository_ctx,
        repository_ctx.attr.version,
    )

    # Collect includes.
    cflags = _exec(python_config, ["--includes"], "include query").split(" ")
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
    linkopts = _exec(python_config, ["--ldflags"], "flag query").split(" ")
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

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by python_repository()

licenses(["notice"])  # Python-2.0 / Python-3.0

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
    """.format(includes, linkopts, linkopts_direct_link)

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

    skylark_content = """
# DO NOT EDIT: generated by python_repository()
# WARNING: Do NOT use this macro file in any neighboring external repository
# rules.

PY_VERSION = "{version}"
PY_SITE_PACKAGES_RELPATH = "{site_packages_relpath}"
""".format(
        version = py_info.version,
        site_packages_relpath = py_info.site_packages_relpath,
    )
    repository_ctx.file(
        "version.bzl",
        content = skylark_content,
        executable = False,
    )

python_repository = repository_rule(
    _impl,
    attrs = {"version": attr.string(default = "2")},
    local = True,
)
