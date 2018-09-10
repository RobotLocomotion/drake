import os
import sys

from drake.tools.lint.formatter import IncludeFormatter


def _check_invalid_line_endings(filename):
    """Return 0 if all of the newlines in @p filename are Unix, and 1
    otherwise.
    """
    # Ask Python to read the file and determine the newlines convention.
    with open(filename, 'rU') as file:
        if not file:
            print("error: unable to open " + filename)
            return 1
        file.read()
        if file.newlines is None:
            newlines = tuple()
        else:
            newlines = tuple(file.newlines)

    # Only allow Unix newlines.
    for newline in newlines:
        if newline != '\n':
            print("error: non-Unix newline characters found")
            return 1

    return 0


def _check_includes(filename):
    """Return 0 if clang-format-includes is a no-op, and 1 otherwise."""
    tool = IncludeFormatter(filename)
    tool.format_includes()
    first_difference = tool.get_first_differing_original_index()
    if first_difference is not None:
        print("error: " + filename + ":" + str(first_difference + 1) + ": " +
              "the #include ordering is incorrect")
        print("note: fix via bazel-bin/tools/lint/clang-format-includes " +
              filename)
        return 1
    return 0


def _check_shebang(filename):
    """Return 0 if the filename's executable bit is consistent with the
    presence of a shebang line and the shebang line is in the whitelist of
    acceptable shebang lines, and 1 otherwise.
    """
    is_executable = os.access(filename, os.X_OK)
    with open(filename, 'r') as file:
        shebang = file.readline().rstrip("\n")
        has_shebang = shebang.startswith("#!")
    if is_executable and not has_shebang:
        print("error: {} is executable but lacks a shebang".format(filename))
        print("note: fix via chmod a-x '{}'".format(filename))
        return 1
    if has_shebang and not is_executable:
        print("error: {} has a shebang but is not executable".format(filename))
        print("note: fix by removing the first line of the file")
        return 1
    shebang_whitelist = {
        "bash": "#!/bin/bash",
        "directorPython": "#!/usr/bin/env directorPython",
        "python": "#!/usr/bin/env python2"
    }
    if has_shebang and shebang not in shebang_whitelist.values():
        print(("error: shebang '{}' in the file '{}' is not in the shebang "
              "whitelist").format(shebang, filename))
        for hint, replacement_shebang in shebang_whitelist.iteritems():
            if hint in shebang:
                print(("note: fix by replacing the shebang with "
                      "'{}'").format(replacement_shebang))
        return 1
    return 0


def main():
    """Run Drake lint checks on each path specified as a command-line argument.
    Exit 1 if any of the paths are invalid or any lint checks fail.
    Otherwise exit 0.
    """
    total_errors = 0
    for filename in sys.argv[1:]:
        print("drakelint.py: Linting " + filename)
        total_errors += _check_invalid_line_endings(filename)
        if not filename.endswith((".cc", ".cpp", ".h")):
            # TODO(jwnimmer-tri) We should enable this check for C++ files
            # also, but that runs into some struggle with genfiles.
            total_errors += _check_shebang(filename)
        if not filename.endswith(".py"):
            total_errors += _check_includes(filename)

    if total_errors == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
