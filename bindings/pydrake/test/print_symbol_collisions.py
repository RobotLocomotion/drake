"""
Prints all symbol collisions that may happen due to pydrake.all.
"""

from collections import defaultdict
import sys

# Populate `sys.modules`.
import pydrake.all


def _is_module_of(name, target):
    return name == target or name.startswith(target + ".")


def _is_public_module(name):
    return name[0] != "_" and "._" not in name


def _get_all_var_names(m):
    # Represents names seen by `from m import *`.
    var_names = getattr(m, "__all__", None)
    if var_names is None:
        var_names = [x for x in dir(m) if not x[0] == "_"]
    return var_names


def main():
    name_to_modules = defaultdict(set)
    # Use id() just in case type is not hashable.
    name_to_value_ids = defaultdict(set)
    for module_name, m in sys.modules.items():
        if (
            not _is_module_of(module_name, "pydrake")
            or not _is_public_module(module_name)
        ):
            continue
        for var_name in _get_all_var_names(m):
            value = getattr(m, var_name)
            value_id = id(value)
            value_ids = name_to_value_ids[var_name]
            if value_id not in value_ids:
                # Should track, as this is a newly seen value.
                value_ids.add(value_id)
                name_to_modules[var_name].add(module_name)
    conflicts = []
    for var_name, module_names in name_to_modules.items():
        assert len(module_names) > 0
        if len(module_names) == 1:
            continue
        conflicts.append((var_name, module_names))

    assert len(conflicts) > 0, "There should be at least some conflicts!"
    print("Conflicts:")
    for var_name, module_names in sorted(conflicts):
        print(f"  '{var_name}' has distinct values in")
        for module_name in sorted(module_names):
            print(f"    {module_name}")


if __name__ == "__main__":
    main()
