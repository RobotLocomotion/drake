import argparse


def _rewrite_one(*, args, old_filename, new_filename):
    """Read in old_filenamd and write into new_filename with specific
    alterations:
    - TBD
    """
    # Read the original.
    with open(old_filename, 'r', encoding='utf-8') as in_file:
        text = in_file.read()

    # Re-spell the project's own include statements.
    for old_inc, new_inc in args.include_edit:
        text = text.replace(f'#include "{old_inc}', f'#include "{new_inc}')

    # Add an inline namespace around the whole file, but disable it around
    # include statements.
    open_inline = 'inline namespace drake_vendor {\n'
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

    # Write out the altered file.
    with open(new_filename, 'w', encoding='utf-8') as out_file:
        out_file.write(text)


def _split_pair(arg):
    old, new = arg.split(':')
    return (old, new)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--include-edit', action='append', type=_split_pair,
        help='Project-local include spellings rewrite')
    parser.add_argument(
        'rewrite', nargs='+', type=_split_pair,
        help='Filename pairs to rewrite, given as IN:OUT')
    args = parser.parse_args()
    for old, new in args.rewrite:
        _rewrite_one(args=args, old_filename=old, new_filename=new)


assert __name__ == '__main__'
main()
