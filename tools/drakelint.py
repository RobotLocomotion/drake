import sys


def _check_invalid_line_endings(newlines):
    """Return 0 if all of the @p newlines are Unix, and 1 otherwise."""
    for newline in newlines:
        if newline != '\n':
            print("Non-Unix newline characters found.")
            return 1
    return 0


def main():
    """Run Drake lint checks on each path specified as a command-line argument.
    Exit 1 if any of the paths are invalid or any lint checks fail.
    Otherwise exit 0.
    """
    total_errors = 0
    for filename in sys.argv[1:]:
        # Open the file.
        print("drakelint.py: Linting " + filename)
        with open(filename, 'rU') as file:
            if not file:
                print("Unable to open " + filename)
                sys.exit(1)

            # Read its data.
            file.read()
            if file.newlines is None:
                newlines = tuple()
            else:
                newlines = tuple(file.newlines)

            # Run tests and count errors.
            errors = 0
            errors += _check_invalid_line_endings(newlines)

            if errors != 0:
                print(str(errors) + " errors detected.")

            # Wrap up.
            total_errors += errors

    if total_errors == 0:
        sys.exit(0)
    else:
        sys.exit(1)


if __name__ == "__main__":
    main()
