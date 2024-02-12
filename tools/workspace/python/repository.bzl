"""
Finds local system Python headers and libraries using python-config and makes
them available to be used as a C/C++ dependency.

There are two available targets:

(1) "@foo//:python" for the typical case where we are creating loadable dynamic
libraries to be used as Python modules.

(2) "@foo//:python_direct_link" for the unusual case of linking the CPython
interpreter as a library into an executable with a C++ main() function.

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
    linux_interpreter_path: (Optional) Interpreter path for the Python runtime,
        when running on Linux.
    macos_interpreter_path: (Optional) Interpreter path for the Python runtime,
        when running on macOS. The format substitution "{homebrew_prefix}" is
        available for use in this string.
"""

load(
    "//tools/workspace:execute.bzl",
    "execute_or_fail",
    "homebrew_prefix",
    "which",
)

def repository_python_info(repository_ctx):
    """Determines the following information:
    - `python` - binary path
    - `python_config` - configuration binary path
    - `version` - '{major}.{minor}'
    - `version_major` - major version (as a string)
    - `site_packages_relpath` - relative to base of FHS
    and returns those details in a struct().
    """

    # - `python`
    # - `python_config`
    if repository_ctx.os.name == "mac os x":
        python = repository_ctx.attr.macos_interpreter_path
        if "{homebrew_prefix}" in python:
            python = python.format(
                homebrew_prefix = homebrew_prefix(repository_ctx),
            )
    else:
        python = repository_ctx.attr.linux_interpreter_path
    python_config = "{}-config".format(python)

    # - `version`
    # - `version_major`
    # - `site_packages_relpath`
    version = execute_or_fail(repository_ctx, [python, "-c", "\n".join([
        "from sys import version_info as v",
        "print('{}.{}'.format(v.major, v.minor))",
    ])]).stdout.strip()
    version_major, _ = version.split(".")
    site_packages_relpath = "lib/python{}/site-packages".format(version)

    # Validate the `python` and `python_config` binaries.
    implementation = execute_or_fail(repository_ctx, [python, "-c", "\n".join([
        "import platform",
        "print(platform.python_implementation())",
    ])]).stdout.strip()
    if implementation != "CPython":
        fail(("The implementation of '{}' is '{}', but only 'CPython' is " +
              "supported.").format(python, implementation))
    if which(repository_ctx, python_config) == None:
        # Developer note: If you are using a `virtualenv` (which is officially
        # unsupported), ensure that you manually symlink the `python3-config`
        # binary in your `virtualenv` installation.
        fail(("Cannot find corresponding config executable: {}\n" +
              "  From interpreter: {}").format(python_config, python))

    return struct(
        python = python,
        python_config = python_config,
        version = version,
        version_major = version_major,
        site_packages_relpath = site_packages_relpath,
    )

def _impl(repository_ctx):
    # Repository implementation.
    py_info = repository_python_info(repository_ctx)

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

    # Prepare the include paths.
    includes = []
    for cflag in cflags:
        if cflag.startswith("-I"):
            source = repository_ctx.path(cflag[2:])
            destination = base.get_child(str(source).replace("/", "_"))
            include = str(destination)[root_len:]
            if include not in includes:
                repository_ctx.symlink(source, destination)
                includes += [include]

    # Collect Python's requested linker options, split on whitespace.
    linkopts = execute_or_fail(
        repository_ctx,
        [py_info.python_config, "--ldflags"],
    ).stdout.strip().split(" ")
    linkopts = [linkopt for linkopt in linkopts if linkopt]

    # Undo whitespace splits for options with a positional argument, e.g., we
    # want ["-framework CoreFoundation"] not ["-framework", "CoreFoundation"].
    for i in reversed(range(len(linkopts))):
        if not linkopts[i].startswith("-"):
            linkopts[i - 1] += " " + linkopts.pop(i)

    # Duplicate ["-Lfoo"] into ["-Wl,-rpath,foo", "-Lfoo"].
    for i in reversed(range(len(linkopts))):
        link_prefix = "-L"
        if linkopts[i].startswith(link_prefix):
            path = linkopts[i][len(link_prefix):]
            linkopts.insert(i, "-Wl,-rpath," + path)

    # Now specialize the the linker options based on whether we're linking a
    # loadable module or an embedded interpreter. (For details, refer to the
    # docs for option (1) vs (2) atop this file.)
    linkopts_embedded = list(linkopts)
    linkopts_embedded.insert(0, "-lpython" + py_info.version)
    linkopts_module = list(linkopts)
    if repository_ctx.os.name == "mac os x":
        linkopts_module.insert(0, "-undefined dynamic_lookup")

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

    build_content = """# DO NOT EDIT: generated by python_repository()

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
    includes = {includes},
    visibility = ["//visibility:private"],
)

cc_library(
    name = "python",
    linkopts = {linkopts_module},
    visibility = ["//visibility:public"],
    deps = [":python_headers"],
)

cc_library(
    name = "python_direct_link",
    linkopts = {linkopts_embedded},
    visibility = ["//visibility:public"],
    deps = [":python_headers"],
)
""".format(
        includes = includes,
        linkopts_module = linkopts_module,
        linkopts_embedded = linkopts_embedded,
    )

    repository_ctx.file(
        "BUILD.bazel",
        content = build_content,
        executable = False,
    )

interpreter_path_attrs = {
    "linux_interpreter_path": attr.string(
        default = "/usr/bin/python3",
    ),
    "macos_interpreter_path": attr.string(
        default = "{homebrew_prefix}/bin/python3.11",
    ),
}

python_repository = repository_rule(
    _impl,
    attrs = interpreter_path_attrs,
    local = True,
    configure = True,
)
