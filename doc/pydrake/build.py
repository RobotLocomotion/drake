"""Command-line tool to generate Drake's Python API reference.

For instructions, see https://drake.mit.edu/documentation_instructions.html.
"""

import os
from os.path import join
import sys

from drake.doc.defs import check_call, main, symlink_input, verbose

import pydrake.all
# TODO(eric.cousineau): Make an optional `.all` module.
from pydrake.examples import (
    acrobot,
    compass_gait,
    manipulation_station,
    pendulum,
    quadrotor,
    rimless_wheel,
    van_der_pol,
)
from pydrake.examples.multibody import cart_pole_passive_simulation
# TODO(eric.cousineau): Indicate these as deprecated.
from pydrake.common import (
    cpp_param,
    cpp_template,
)


def _get_submodules(name):
    prefix = name + "."
    result = []
    for s_name in sys.modules.keys():
        if not s_name.startswith(prefix):
            continue
        sub = s_name[len(prefix):]
        # Ensure its an immediate child.
        if "." in sub or sub.startswith("_"):
            continue
        # For some reason, things like `pydrake.common` has submodules like
        # `inspect`, etc, whose value in `sys.modules` are none. Ignore those.
        # TODO(eric.cousineau): Figure out where these come from, and remove
        # them.
        if sys.modules[s_name] is None:
            continue
        result.append(s_name)
    return sorted(result)


def _get_pydrake_modules():
    """Returns a list[str] of all pydrake modules that should appear in our
    Python API reference.
    """
    result = []
    worklist = ["pydrake"]
    while worklist:
        current = worklist.pop(0)
        result.append(current)
        for sub in _get_submodules(current):
            worklist.append(sub)
    return sorted(result)


def _has_cc_imported_symbols(name):
    # Check for `module_py`.
    if name + "._module_py" in sys.modules:
        return True
    pieces = name.split(".")
    if len(pieces) > 1:
        sub = pieces[-1]
        test = ".".join(pieces[:-1] + ["_{}_py".format(sub)])
        if test in sys.modules:
            raise RuntimeError(
                ("The module `{}` should not exist; instead, only `{}` should "
                 "exist").format(test, name))
    return False


def _write_module(name, f_name):
    """Writes an rst file for module `name` into `f_name`.
    """
    if verbose():
        print("Write: {}".format(name))
    subs = _get_submodules(name)
    with open(f_name, 'w') as f:
        f.write("\n")
        rst_name = name.replace("_", "\\_")
        f.write("{}\n".format(rst_name))
        f.write("=" * len(rst_name) + "\n")
        f.write("\n")
        if len(subs) > 0:
            f.write(".. toctree::\n")
            f.write("    :maxdepth: 1\n")
            f.write("\n")
            for sub in subs:
                f.write("    {}\n".format(sub))
            f.write("\n\n")
        f.write(".. automodule:: {}\n".format(name))
        f.write("    :members:\n")
        # See if there's a corresponding private CcPybind library.
        if _has_cc_imported_symbols(name):
            f.write("    :imported-members:\n")
        f.write("    :undoc-members:\n")
        f.write("    :show-inheritance:\n")


def _build(*, out_dir, temp_dir, modules):
    """Generates into out_dir; writes scratch files into temp_dir.
    As a precondition, both directories must already exist and be empty.
    If modules are provided, only generate those modules and their children.
    """
    assert len(os.listdir(temp_dir)) == 0
    assert len(os.listdir(out_dir)) == 0

    sphinx_build = "/usr/share/sphinx/scripts/python3/sphinx-build"
    assert os.path.isfile(sphinx_build)

    # Create a hermetic copy of our input.  This helps ensure that only files
    # listed in BUILD.bazel will render onto the website.
    symlink_input(
        "drake/doc/pydrake/sphinx_input.txt", temp_dir,
        strip_prefix=["drake/doc/"])
    input_dir = join(temp_dir, "pydrake")

    # Process the command-line request for which modules to document.
    all_modules = _get_pydrake_modules()
    if not modules:
        modules_to_document = set(all_modules)
    else:
        modules_to_document = set()
        for x in modules:
            if x not in all_modules:
                print(f"error: Unknown module '{x}'")
                sys.exit(1)
            # Add the requested module and its parents.
            tokens = x.split(".")
            while tokens:
                modules_to_document.add(".".join(tokens))
                tokens.pop()
            # Add the requsted module's children.
            for y in all_modules:
                if y.startswith(x + "."):
                    modules_to_document.add(y)

    # Generate tables of contents.
    for name in sorted(list(modules_to_document)):
        if name == "pydrake":
            rst_name = "index.rst"
        else:
            rst_name = name + ".rst"
        _write_module(name, join(input_dir, rst_name))

    # Run the documentation generator.
    os.environ["LANG"] = "en_US.UTF-8"
    check_call([
        sphinx_build,
        "-b", "html",  # HTML output.
        "-a", "-E",  # Don't use caching.
        "-N",  # Disable colored output.
        "-T",  # Traceback (for plugin).
        "-d", join(temp_dir, "doctrees"),
        input_dir,
        out_dir,
    ])

    # The filename to suggest as the starting point for preview; in this case,
    # it's an empty filename (i.e., the index page).
    return [""]


# TODO(eric.cousineau): Do some simple linting if this is run under `bazel
# test` (e.g. scan for instances of `TemporaryName`, scan for raw C++ types
# in type signatures, etc).
if __name__ == "__main__":
    main(build=_build, subdir="pydrake", description=__doc__.strip(),
         supports_modules=True)
