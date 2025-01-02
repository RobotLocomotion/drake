"""
This rule configures Python for use by Drake, in two parts:
(a) Finds local system Python headers and libraries using python-config and
    makes them available to be used as a C/C++ dependency.
(b) macOS only: creates (or syncs) a virtual environment for dependencies.

For part (a) there are two available targets:

(1) "@python//:python" for the typical case where we are creating loadable
dynamic libraries to be used as Python modules.

(2) "@python//:python_direct_link" for the unusual case of linking the CPython
interpreter as a library into an executable with a C++ main() function.

For part (b) the environment is used in all python rules and tests by default,
because the python toolchain's interpreter is the venv python3 binary.

If the {macos,linux}_interpreter_path being used only mentions the python major
version (i.e., it is "/path/to/python3" not "/path/to/python3.##") and if the
interpreter is changed to a different minor version without any change to the
path, then you must run `bazel sync --configure` to re-run this repository rule
in order to make bazel aware of the new minor version. This hazard cannot occur
on any of Drake's supported platforms with our default values, but if you are
trying something out of the ordinary, be aware.

Arguments:
    name: A unique name for this rule.
    linux_interpreter_path: (Optional) Interpreter path for the Python runtime,
        when running on Linux.
    macos_interpreter_path: (Optional) Interpreter path for the Python runtime,
        when running on macOS. The format substitution "{homebrew_prefix}" is
        available for use in this string.
    requirements_flavor: (Optional) Which Python dependencies to install.
"""

load(
    "//tools/workspace:execute.bzl",
    "execute_or_fail",
    "homebrew_prefix",
    "which",
)

def _get_python_interpreter(repo_ctx):
    """Returns the tuple (python_interpreter_path, major_minor_version) based
    on the settings in our repository rule's attrs.
    """
    if repo_ctx.os.name == "mac os x":
        python = repo_ctx.attr.macos_interpreter_path
        if "{homebrew_prefix}" in python:
            python = python.format(
                homebrew_prefix = homebrew_prefix(repo_ctx),
            )
    else:
        python = repo_ctx.attr.linux_interpreter_path
    implementation = execute_or_fail(repo_ctx, [python, "-c", "\n".join([
        "import platform",
        "print(platform.python_implementation())",
    ])]).stdout.strip()
    if implementation != "CPython":
        fail(("The implementation of '{}' is '{}', but only 'CPython' is " +
              "supported.").format(python, implementation))
    version = execute_or_fail(repo_ctx, [python, "-c", "\n".join([
        "from sys import version_info as v",
        "print('{}.{}'.format(v.major, v.minor))",
    ])]).stdout.strip()
    return (python, version)

def _get_extension_suffix(repo_ctx, python, python_config):
    """Returns the extension suffix, e.g. ".cpython-310-x86_64-linux-gnu.so" as
    queried from python_config. Uses `python` only for error reporting.
    """
    if which(repo_ctx, python_config) == None:
        fail(("Cannot find corresponding config executable: {}\n" +
              "  From interpreter: {}").format(python_config, python))
    return execute_or_fail(
        repo_ctx,
        [python_config, "--extension-suffix"],
    ).stdout.strip()

# TODO(jwnimmer-tri): Much of the logic for parsing includes and linkopts is
# the same or similar to that used in pkg_config.bzl and should be refactored
# and shared instead of being duplicated in both places.

def _get_includes(repo_ctx, python_config):
    """Returns the list of `includes = ...` when compiling native code."""
    includes = []
    cflags = execute_or_fail(
        repo_ctx,
        [python_config, "--includes"],
    ).stdout.strip().split(" ")
    for cflag in cflags:
        if cflag.startswith("-I"):
            include = "include/" + cflag[2:].replace("/", "_")
            if include not in includes:
                repo_ctx.symlink(cflag[2:], include)
                includes.append(include)
    return includes

def _get_linkopts(repo_ctx, python_config):
    """Returns the list of `linkopts = ...` when compiling native code."""

    # Collect Python's requested linker options, split on whitespace.
    linkopts = execute_or_fail(
        repo_ctx,
        [python_config, "--ldflags"],
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
    return linkopts

def _prepare_venv(repo_ctx, python):
    # Only macOS uses a venv at the moment.
    os_name = repo_ctx.os.name  # "linux" or "mac os x"
    if os_name != "mac os x":
        return python

    # Locate the lock file and mark it to be monitored for changes.
    pylock = repo_ctx.path(Label("@drake//setup:python/pdm.lock")).realpath
    repo_ctx.symlink(pylock, "pdm.lock")
    repo_ctx.watch(pylock)

    # Choose which dependencies to install.
    if repo_ctx.attr.requirements_flavor == "test":
        repo_ctx.file("@pdm-install-args", content = "-G test")
    else:
        repo_ctx.file("@pdm-install-args", content = "--prod")

    # Run venv_sync to ensure the venv content matches the pdm.lock; it will
    # (un)install any packages as necessary, or even create the venv when it
    # doesn't exist at all yet.
    sync_label = Label("@drake//tools/workspace/python:venv_sync")
    sync = repo_ctx.path(sync_label).realpath
    repo_ctx.report_progress("Running venv_sync")
    execute_or_fail(repo_ctx, [
        sync,
        "--python",
        python,
        "--repository",
        repo_ctx.path("").realpath,
    ])
    repo_ctx.watch(sync)

    # Read the path to the venv's python3. (This file is created by venv_sync.)
    venv_python3 = repo_ctx.read("venv_python3.txt")
    return repo_ctx.path(venv_python3)

def _impl(repo_ctx):
    # Add the BUILD file.
    repo_ctx.symlink(
        Label("//tools/workspace/python:package.BUILD.bazel"),
        "BUILD.bazel",
    )

    # Set `python` to the the interpreter path specified by our rule attrs,
    # and `version` to its "major.minor" string.
    python, version = _get_python_interpreter(repo_ctx)
    site_packages_relpath = "lib/python{}/site-packages".format(version)

    # Get extension_suffix, includes, and linkopts from python_config.
    python_config = "{}-config".format(python)
    extension_suffix = _get_extension_suffix(repo_ctx, python, python_config)
    includes = _get_includes(repo_ctx, python_config)
    linkopts = _get_linkopts(repo_ctx, python_config)

    # Specialize the the linker options based on whether we're linking a
    # loadable module or an embedded interpreter. (For details, refer to
    # the docs for option (1) vs (2) atop this file.)
    linkopts_embedded = list(linkopts)
    linkopts_embedded.insert(0, "-lpython" + version)
    linkopts_module = list(linkopts)
    if repo_ctx.os.name == "mac os x":
        linkopts_module.insert(0, "-undefined dynamic_lookup")

    # Set up (or sync) the venv.
    bin_path = _prepare_venv(repo_ctx, python)

    version_content = """
# DO NOT EDIT: generated by python_repository()
# WARNING: Avoid using this macro in any repository rules which require
# `load()` at the WORKSPACE level. Instead, load these constants through
# `BUILD.bazel` or `package.BUILD.bazel` files.

PYTHON_BIN_PATH = "{bin_path}"
PYTHON_EXTENSION_SUFFIX = "{extension_suffix}"
PYTHON_VERSION = "{version}"
PYTHON_SITE_PACKAGES_RELPATH = "{site_packages_relpath}"
PYTHON_INCLUDES = {includes}
PYTHON_LINKOPTS_EMBEDDED = {linkopts_embedded}
PYTHON_LINKOPTS_MODULE = {linkopts_module}
""".format(
        bin_path = bin_path,
        extension_suffix = extension_suffix,
        version = version,
        site_packages_relpath = site_packages_relpath,
        includes = includes,
        linkopts_module = linkopts_module,
        linkopts_embedded = linkopts_embedded,
    )
    repo_ctx.file(
        "version.bzl",
        content = version_content,
        executable = False,
    )

interpreter_path_attrs = {
    "linux_interpreter_path": attr.string(
        default = "/usr/bin/python3",
    ),
    "macos_interpreter_path": attr.string(
        # The version listed here should match what's listed in both the root
        # CMakeLists.txt and doc/_pages/installation.md.
        default = "{homebrew_prefix}/bin/python3.12",
    ),
    "requirements_flavor": attr.string(
        default = "test",
        values = ["build", "test"],
    ),
}

python_repository = repository_rule(
    _impl,
    attrs = interpreter_path_attrs,
    configure = True,
)
