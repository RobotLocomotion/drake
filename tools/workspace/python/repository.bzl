"""
This rule configures Python for use by Drake, in two parts:
(a) Finds local system Python headers and libraries using python-config and
    makes them available to be used as a C/C++ dependency.
(b) macOS only: creates (or syncs) a virtual environment for dependencies.

For part (a) our goal is to create a @python//:version.bzl file with the
details of what we found, which is loaded by package.BUILD.bazel to define
the @python//:all toolchains, which are loaded by our tools/bazel.rc.

Anywhere in Drake that needs to *consume* the python toolchain information
should use the aliases provided by //tools/workspace/python which always cite
the current toolchain (which might not be Drake's toolchain in case a user
configured a different toolchain). Nothing in Drake should ever refer to any
@python//:foo label directly; everything must always happen via toolchain.

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

    # Opt-out of support for the (deprecated) _crypt module. Linking it just
    # bloats our wheels for no good reason. Once Python 3.13 is our minimum
    # supported version, we can remove this stanza.
    if "-lcrypt" in linkopts:
        linkopts.remove("-lcrypt")

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
    # Only macOS and wheel builds use a venv at the moment.
    os_name = repo_ctx.os.name  # "linux" or "mac os x"
    is_wheel_build = repo_ctx.getenv("DRAKE_WHEEL", "") == "1"
    if os_name != "mac os x" and not is_wheel_build:
        return python

    # Locate lock files and mark them to be monitored for changes.
    requirements = repo_ctx.path(
        Label("@drake//setup:python/requirements.txt"),
    ).realpath
    pdmlock = repo_ctx.path(Label("@drake//setup:python/pdm.lock")).realpath
    repo_ctx.watch(requirements)
    repo_ctx.watch(pdmlock)

    # Choose which dependencies to install.
    if is_wheel_build:
        repo_ctx.file("@pdm-install-args", content = "-G wheel")
    elif repo_ctx.attr.requirements_flavor == "test":
        repo_ctx.file("@pdm-install-args", content = "-G test")
    else:
        repo_ctx.file("@pdm-install-args", content = "--prod")

    # Run venv_sync to ensure the venv content matches the pdm.lock; it will
    # (un)install any packages as necessary, or even create the venv when it
    # doesn't exist at all yet.
    sync_label = Label("@drake//tools/workspace/python:venv_sync")
    sync = repo_ctx.path(sync_label).realpath
    sync_args = [
        "--python",
        python,
        "--repository",
        repo_ctx.path("").realpath,
    ]
    if is_wheel_build:
        sync_args.append("--symlink")
    repo_ctx.report_progress("Running venv_sync")
    execute_or_fail(repo_ctx, [sync] + sync_args)
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

    # Get includes and linkopts from python_config.
    python_config = "{}-config".format(python)
    includes = _get_includes(repo_ctx, python_config)
    linkopts = _get_linkopts(repo_ctx, python_config)

    # On macOS, we need to tweak the linkopts slightly.
    if repo_ctx.os.name == "mac os x":
        linkopts.insert(0, "-undefined dynamic_lookup")

    # Set up (or sync) the venv.
    bin_path = _prepare_venv(repo_ctx, python)

    version_content = """
PYTHON_BIN_PATH = {bin_path}
PYTHON_VERSION = {version}
PYTHON_INCLUDES = {includes}
PYTHON_LINKOPTS = {linkopts}
""".format(
        bin_path = repr(bin_path),
        version = repr(version),
        includes = repr(includes),
        linkopts = repr(linkopts),
    )
    repo_ctx.file(
        "version.bzl",
        content = version_content,
        executable = False,
    )

python_repository = repository_rule(
    _impl,
    environ = [
        "DRAKE_WHEEL",
    ],
    attrs = {
        "linux_interpreter_path": attr.string(
            default = "/usr/bin/python3",
        ),
        "macos_interpreter_path": attr.string(
            # When changing this, see drake/tools/workspace/python/README.md.
            default = "{homebrew_prefix}/bin/python3.14",
        ),
        "requirements_flavor": attr.string(
            default = "test",
            values = ["build", "test"],
        ),
    },
    configure = True,
)
