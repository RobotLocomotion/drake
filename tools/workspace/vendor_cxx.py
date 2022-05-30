"""Build system tool that transforms C++ source code for easy vendoring.

Rewrites the include statements, namespace, and symbol visibility with the goal
of producing a completely independent build of some upstream library, even when
statically linking other versions of the library into the same DSO.

Note that this works only on C++ code, not plain C code.
"""

import argparse


def _rewrite_one_text(*, text, edit_include):
    """Rewrites the C++ file contents in `text` with specific alterations:

    - The paths in #include statements are replaced per the (old, new) pairs in
    the include_edit list. Only includes that use quotation marks will be
    changed. The pairs are literal strings that must be a prefix of the path.

    - Wraps an inline namespace "drake_vendor" with hidden symbol visibility
    around the entire file; it is withdrawn prior to any include statement.

    Returns the new C++ contents.

    These changes should suffice for the most typical flavors of C++ code.
    Tricks like including non-standalone files (`#include "helpers.inc"`)
    may not work.
    """
    # Re-spell the project's own include statements.
    for old_inc, new_inc in edit_include:
        text = text.replace(f'#include "{old_inc}', f'#include "{new_inc}')

    # Add an inline namespace around the whole file, but disable it around
    # include statements.
    open_inline = ' '.join([
        'inline namespace drake_vendor',
        '__attribute__ ((visibility ("hidden")))',
        '{\n'])
    close_inline = '}  /* inline namespace drake_vendor */\n'
    text = open_inline + text + close_inline
    search_start = 0
    while True:
        i = text.find('\n#include ', search_start)
        if i < 0:
            break
        first = i + 1
        last = text.index('\n', first) + 1
        text = (
            text[:first]
            + close_inline
            + text[first:last]
            + open_inline
            + text[last:])
        search_start = last + len(open_inline) + len(close_inline) - 1

    return text


def _rewrite_one_file(*, old_filename, new_filename, edit_include):
    """Reads in old_filename and write into new_filename with specific
    alterations as described by _rewrite_one_string().
    """
    # Read the original.
    with open(old_filename, 'r', encoding='utf-8') as in_file:
        old_text = in_file.read()

    new_text = _rewrite_one_text(text=old_text, edit_include=edit_include)

    # Write out the altered file.
    with open(new_filename, 'w', encoding='utf-8') as out_file:
        out_file.write(new_text)


def _split_pair(arg):
    """Helper function to split ':'-delimited pairs on the command line.
    """
    old, new = arg.split(':')
    return (old, new)


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--edit-include', action='append', default=[],
        type=_split_pair, metavar='OLD:NEW',
        help='Project-local include spellings rewrite')
    parser.add_argument(
        'rewrite', nargs='+', type=_split_pair,
        help='Filename pairs to rewrite, given as IN:OUT')
    args = parser.parse_args()
    for old_filename, new_filename in args.rewrite:
        _rewrite_one_file(
            edit_include=args.edit_include,
            old_filename=old_filename,
            new_filename=new_filename)


if __name__ == '__main__':
    _main()
