"""Refreshes *.rst files for Python modules."""

import argparse
import glob
import os
from os.path import dirname, isabs, isfile, join
import sys

import pydrake.all
# TODO(eric.cousineau): Make an optional `.all` module.
from pydrake.examples import (
    acrobot,
    compass_gait,
    pendulum,
    rimless_wheel,
    van_der_pol,
)
from pydrake.examples.multibody import cart_pole_passive_simulation
# TODO(eric.cousineau): Indicate these as deprecated.
from pydrake.util import (
    cpp_const,
    cpp_param,
    cpp_template,
)

EXCLUDE = [
    "pydrake.third_party",
]


def get_submodules(name):
    prefix = name + "."
    out = []
    for s_name in sys.modules.keys():
        if s_name in EXCLUDE:
            continue
        if not s_name.startswith(prefix):
            continue
        sub = s_name[len(prefix):]
        # Ensure its an immediate child.
        if "." in sub or sub.startswith("_"):
            continue
        # For some reason, things like `pydrake.util` has submodules like
        # `inspect`, etc, whose value in `sys.modules` are none. Ignore those.
        # TODO(eric.cousineau): Figure out where these come from, and remove
        # them.
        if sys.modules[s_name] is None:
            continue
        out.append(s_name)
    return sorted(out)


def has_cc_imported_symbols(name):
    # Check for `module_py`.
    if name + "._module_py" in sys.modules:
        return True
    pieces = name.split(".")
    if len(pieces) > 1:
        sub = pieces[-1]
        test = ".".join(pieces[:-1] + ["_{}_py".format(sub)])
        if test in sys.modules:
            return True
    return False


def write_module(f_name, name, verbose):
    if verbose:
        print("Write: {}".format(name))
    subs = get_submodules(name)
    with open(f_name, 'w') as f:
        f.write(".. GENERATED FILE DO NOT EDIT\n")
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
        if has_cc_imported_symbols(name):
            f.write("    :imported-members:\n")
        f.write("    :undoc-members:\n")
        f.write("    :show-inheritance:\n")
    f_dir = dirname(f_name)
    for sub in subs:
        write_module(join(f_dir, sub) + ".rst", sub, verbose)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output_dir", type=str, required=True,
        help="Output directory; must be an absolute path, and must already "
             "exist.")
    parser.add_argument(
        "--pre_clean", action="store_true",
        help="Remove `index.rst` and all `pydrake.*.rst` files before "
             "generating.")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    output_dir = args.output_dir
    if not isabs(output_dir):
        sys.stderr.write(
            "Please provide an absolute path.\n")
        sys.exit(1)
    pre_clean = args.pre_clean
    verbose = args.verbose

    index_file = join(output_dir, "index.rst")
    if pre_clean:
        old_files = glob.glob(join(output_dir, "pydrake.*.rst"))
        if isfile(index_file):
            old_files.append(index_file)
        for old_file in sorted(old_files):
            if verbose:
                print("Remove: {}".format(old_file))
            os.remove(old_file)
    write_module(index_file, "pydrake", verbose)


if __name__ == "__main__":
    main()
