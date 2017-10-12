import sys

from tools.lint.formatter import IncludeFormatter


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


def main():
    """Run Drake lint checks on each path specified as a command-line argument.
    Exit 1 if any of the paths are invalid or any lint checks fail.
    Otherwise exit 0.
    """
    total_errors = 0
    for filename in sys.argv[1:]:
        print("drakelint.py: Linting " + filename)
        total_errors += _check_invalid_line_endings(filename)
        total_errors += _check_includes(filename)

    if total_errors == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
