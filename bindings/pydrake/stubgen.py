"""Command-line tool to generate Drake's Python type-completion."""

import sys
import pybind11_stubgen

import pydrake.all


def _modules_to_scan():
    names = ["pydrake"]
    for name in sys.modules.keys():
        if "._" in name:
            # Private module.
            continue
        if name.startswith("pydrake."):
            names.append(name)
    return names


if __name__ == "__main__":
    args = sys.argv[1:]
    modules = _modules_to_scan()

    pybind11_stubgen.main(args + modules)
