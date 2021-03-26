import os
import sys

from drake.tools.lint.formatter import IncludeFormatter


def _check_unguarded_openmp_uses(filename):
    """Return 0 if all OpenMP uses in @p filename are properly guarded by
    #if defined(_OPENMP), and 1 otherwise.
    """
    openmp_include = "#include <omp.h>"
    openmp_pragma = "#pragma omp"

    openmp_pre_guard = "#if defined(_OPENMP)"
    openmp_post_guard = "#endif"

    with open(filename, mode='r', encoding='utf-8') as file:
        lines = file.readlines()

    for index, current_line in enumerate(lines):
        if openmp_include in current_line or openmp_pragma in current_line:
            previous_line = lines[index - 1] if (index - 1) >= 0 else ""
            next_line = lines[index + 1] if (index + 1) < len(lines) else ""

            missing_pre_guard = previous_line.strip() != openmp_pre_guard
            missing_post_guard = next_line.strip() != openmp_post_guard

            if missing_pre_guard or missing_post_guard:
                print(f"ERROR: {filename}:{index + 1}: "
                      "OpenMP includes and directives must be guarded by "
                      f"{openmp_pre_guard} on the previous line and "
                      f"{openmp_post_guard} on the following line")

                return 1
    return 0


def _check_invalid_line_endings(filename):
    """Return 0 if all of the newlines in @p filename are Unix, and 1
    otherwise.
    """
    # Ask Python to read the file and determine the newlines convention.
    with open(filename, mode='r', encoding='utf-8') as file:
        file.read()
        if file.newlines is None:
            newlines = tuple()
        else:
            newlines = tuple(file.newlines)

    # Only allow Unix newlines.
    for newline in newlines:
        if newline != '\n':
            print("ERROR: non-Unix newline characters found")
            return 1

    return 0


def _check_includes(filename):
    """Return 0 if clang-format-includes is a no-op, and 1 otherwise."""
    try:
        tool = IncludeFormatter(filename)
    except Exception as e:
        print("ERROR: " + filename + ":0: " + str(e))
        return 1
    tool.format_includes()
    first_difference = tool.get_first_differing_original_index()
    if first_difference is not None:
        print(f"ERROR: {filename}:{first_difference + 1}: "
              "the #include ordering is incorrect")
        print("note: fix via bazel-bin/tools/lint/clang-format-includes "
              + filename)
        print("note: if that program does not exist, "
              "you might need to compile it first: "
              "bazel build //tools/lint/...")
        return 1
    return 0


def _check_shebang(filename, disallow_executable):
    """Return 0 if the filename's executable bit is consistent with the
    presence of a shebang line and the shebang line is in the whitelist of
    acceptable shebang lines, and 1 otherwise.

    If the string "# noqa: shebang" is present in the file, then this check
    will be ignored.
    """
    with open(filename, mode='r', encoding='utf8') as file:
        content = file.read()
    if "# noqa: shebang" in content:
        # Ignore.
        return 0

    is_executable = os.access(filename, os.X_OK)
    if is_executable and disallow_executable:
        print("ERROR: {} is executable, but should not be".format(filename))
        print("note: fix via chmod a-x '{}'".format(filename))
        return 1

    lines = content.splitlines()
    assert len(lines) > 0, f"Empty file? {filename}"
    shebang = lines[0]
    has_shebang = shebang.startswith("#!")
    if is_executable and not has_shebang:
        print("ERROR: {} is executable but lacks a shebang".format(filename))
        print("note: fix via chmod a-x '{}'".format(filename))
        return 1
    if has_shebang and not is_executable:
        print("ERROR: {} has a shebang but is not executable".format(filename))
        print("note: fix by removing the first line of the file")
        return 1
    shebang_whitelist = {
        "bash": "#!/bin/bash",
        "python": "#!/usr/bin/env python3",
    }
    if has_shebang and shebang not in list(shebang_whitelist.values()):
        print(("ERROR: shebang '{}' in the file '{}' is not in the shebang "
              "whitelist").format(shebang, filename))
        for hint, replacement_shebang in shebang_whitelist.items():
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
    if len(sys.argv) > 1 and sys.argv[1] == "--disallow_executable":
        disallow_executable = True
        filenames = sys.argv[2:]
    else:
        disallow_executable = False
        filenames = sys.argv[1:]
    for filename in filenames:
        print("drakelint.py: Linting " + filename)
        total_errors += _check_invalid_line_endings(filename)
        if not filename.endswith((".cc", ".cpp", ".h")):
            # TODO(jwnimmer-tri) We should enable this check for C++ files
            # also, but that runs into some struggle with genfiles.
            total_errors += _check_shebang(filename, disallow_executable)
        if not filename.endswith(".py"):
            total_errors += _check_includes(filename)
            total_errors += _check_unguarded_openmp_uses(filename)

    if total_errors == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
