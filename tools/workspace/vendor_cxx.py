"""Build system tool that transforms C++ source code for easy vendoring.

Rewrites the include statements, namespace, and symbol visibility with the goal
of producing a completely independent build of some upstream library, even when
statically linking other versions of the library into the same DSO.

Note that this works only on C++ code, not plain C code.
"""

import argparse
from enum import Enum
import re


def _designate_wrapped_lines(lines):
    """Given a list[str] of the lines in a C++ source file, returns a
    list[bool] that is True iff the corresponding line should be part
    of the inline hidden namespace.

    We MUST wrap all C/C++ code. We must NOT wrap #include statements.

    Blank lines and other non-C++ lines such as comments or non-#include
    preprocessor directives can go either way; we'll group them into the
    wrapping status of their neighbors in order to minimize the number of
    added lines.
    """

    class Flag(Enum):
        WRAP = 1
        NO_WRAP = -1
        DONT_CARE = 0

    # Regexs to match various kinds of code patterns.
    is_include = re.compile(r'^\s*#\s*include\s*["<].*$')
    is_preprocessor = re.compile(r"^\s*#.*$")
    is_blank = re.compile(r"^\s*$")
    is_blank_cpp_comment = re.compile(r"^\s*//.*$")
    is_blank_c_comment_begin = re.compile(r"^\s*/\*.*$")
    is_c_comment_end = re.compile(r"^.*\*/\s*(.*)$")

    # Loop over all lines and determine each one's flag.
    flags = [None] * len(lines)
    i = 0
    while i < len(lines):
        line = lines[i]
        # When the prior line has continuation, this line inherits its Flag.
        if i > 0 and lines[i - 1].endswith("\\"):
            flags[i] = flags[i - 1]
            i += 1
            continue
        # We must NOT wrap #include statements.
        if is_include.match(line):
            flags[i] = Flag.NO_WRAP
            i += 1
            continue
        # Other preprocessor directives can go either way.
        if is_preprocessor.match(line):
            flags[i] = Flag.DONT_CARE
            i += 1
            continue
        # Blank lines (or lines that are blank other than their comments)
        # can go either way.
        if is_blank.match(line) or is_blank_cpp_comment.match(line):
            flags[i] = Flag.DONT_CARE
            i += 1
            continue
        # For C-style comments, consume the entire comment block immediately.
        if is_blank_c_comment_begin.match(line):
            first_c_comment_line = i
            while True:
                line = lines[i]
                match = is_c_comment_end.match(line)
                flags[i] = Flag.DONT_CARE
                i += 1
                if match:
                    break
            # If the close-comment marker had code after it, we need to go back
            # and set the entire C-style comment to WRAP.
            (trailing,) = match.groups()
            if trailing:
                for fixup in range(first_c_comment_line, i):
                    flags[fixup] = Flag.WRAP
            continue
        # We MUST wrap all C/C++ code.
        flags[i] = Flag.WRAP
        i += 1

    # We want to insert inline namespaces such that:
    #
    # - all WRAP lines are enclosed;
    # - no NO_WRAP lines are enclosed;
    # - the only DONT_CARE lines enclosed are surrouneded by WRAP.
    #
    # We'll do that by growing the NO_WRAP spans as large as possible.

    # Grow the start-of-file run of NO_WRAP:
    for i in range(len(flags)):
        if flags[i] == Flag.DONT_CARE:
            flags[i] = Flag.NO_WRAP
        else:
            break

    # Grow the end-of-file run of NO_WRAP:
    for i in range(len(flags) - 1, -1, -1):
        if flags[i] == Flag.DONT_CARE:
            flags[i] = Flag.NO_WRAP
        else:
            break

    # Grow any interior regions of NO_WRAP:
    for i in range(len(flags)):
        if flags[i] == Flag.NO_WRAP:
            # Change all of the immediately prior and subsequent homogeneous
            # runs of DONT_CARE to NO_WRAP.
            for j in range(i - 1, -1, -1):
                if flags[j] == Flag.DONT_CARE:
                    flags[j] = Flag.NO_WRAP
                else:
                    break
            for j in range(i + 1, len(flags)):
                if flags[j] == Flag.DONT_CARE:
                    flags[j] = Flag.NO_WRAP
                else:
                    break

    # Anything remaining is DONT_CARE bookended by WRAP, so we'll WRAP it.
    for i in range(len(flags)):
        if flags[i] == Flag.DONT_CARE:
            flags[i] = Flag.WRAP

    # Return True only for the wrapped lines.
    return [x == Flag.WRAP for x in flags]


def _rewrite_one_text(*, text, inline_namespace):
    """Rewrites the C++ file contents in `text` with specific alterations:

    - Wraps an inline namespace "drake_vendor" with hidden symbol visibility
    around all of the code in file (but not any #include statements).

      - Or when inline_namespace is False, simply marks all of the existing
        namespaces as hidden without any extra inline namespace wrapping.
        This does not hide the vendored library as thoroughly (it's still
        a potential ODR conflict during static linking) but has the benefit
        of working on more complicated projects that our wrapping heuristics
        cannot handle.

    Returns the new C++ contents.

    These changes should suffice for the most typical flavors of C++ code.
    Tricks like including non-standalone files (`#include "helpers.inc"`)
    may not work.
    """
    # If the file is a mixed C/C++ header, then we need to leave it alone.
    if '\nextern "C" {\n' in text:
        return text

    # Prepare to edit one line at a time.
    lines = text.split("\n")
    if lines[-1] == "":
        lines.pop()
    hidden = '__attribute__ ((visibility ("hidden")))'

    # If we are only changing namespaces (not adding new ones), do that now:
    if not inline_namespace:
        # Match either 'namespace foo' or 'namespace foo {'.
        regex = re.compile(r"^\s*namespace\s+([^{]+?)(\s*{)?$")
        for i, line in enumerate(lines):
            match = regex.match(line)
            if not match:
                continue
            name, brace = match.groups()
            lines[i] = f"namespace {name} {hidden}{brace or ''}"
        text = "\n".join(lines) + "\n"
        return text

    # We'll add an inline namespace around the C++ code in this file.
    # Designate each line of the file for whether it should be wrapped.
    should_wrap = _designate_wrapped_lines(lines)

    # Anytime the sense of wrapping switches, we'll insert a line.
    # Do this in reverse order so that the indices into lines[] are stable.
    open_inline = " ".join(["inline namespace drake_vendor", hidden, "{"])
    close_inline = "}  /* inline namespace drake_vendor */"
    for i in range(len(lines), -1, -1):
        this_wrap = should_wrap[i] if i < len(lines) else False
        prior_wrap = should_wrap[i - 1] if i > 1 else False
        if this_wrap == prior_wrap:
            continue
        insertion = open_inline if this_wrap else close_inline
        lines.insert(i, insertion)

    text = "\n".join(lines) + "\n"
    return text


def _rewrite_one_file(*, old_filename, new_filename, inline_namespace):
    """Reads in old_filename and write into new_filename with specific
    alterations as described by _rewrite_one_string().
    """
    # Read the original.
    with open(old_filename, "r", encoding="utf-8") as in_file:
        old_text = in_file.read()

    new_text = _rewrite_one_text(
        text=old_text, inline_namespace=inline_namespace
    )

    # Write out the altered file.
    with open(new_filename, "w", encoding="utf-8") as out_file:
        out_file.write(new_text)


def _split_pair(arg):
    """Helper function to split ':'-delimited pairs on the command line."""
    old, new = arg.split(":")
    return (old, new)


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no-inline-namespace",
        dest="inline_namespace",
        action="store_false",
        help="Set visibility directly without an inline namespace wrapper",
    )
    parser.add_argument(
        "rewrite",
        nargs="+",
        type=_split_pair,
        help="Filename pairs to rewrite, given as IN:OUT",
    )
    args = parser.parse_args()
    for old_filename, new_filename in args.rewrite:
        _rewrite_one_file(
            inline_namespace=args.inline_namespace,
            old_filename=old_filename,
            new_filename=new_filename,
        )


if __name__ == "__main__":
    _main()
