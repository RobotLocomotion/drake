#!/usr/bin/env python3

"""
Usage:
  system_doxygen input-file

Will load input-file from disk, replace our custom doxygen @system/@endsystem
tags into html code that can be rendered in the browser, and print the updated
file contents to stdout.

This is intended to be used as an INPUT_FILTER for doxygen.

These utilities are also used by mkdoc.py to handle this custom tag in the
python documentation.
"""

# noqa: shebang
# We suppress shebang lint checking because doxygen must be able to execute
# this file.

import re
import sys
import yaml
from textwrap import indent


def system_yaml_to_html(system_yaml):
    """
    Converts a YAML description of a system into an html table.
    """
    try:
        y = yaml.load(system_yaml, Loader=yaml.SafeLoader)
    except yaml.scanner.ScannerError as e:
        print(system_yaml, file=sys.stderr)
        raise(e)

    input_port_html = ""
    if "input_ports" in y:
        for p in y["input_ports"]:
            input_port_html += (
                f'<tr><td align=right style=\"padding:5px 0px 5px 0px\">{p}'
                f'&rarr;</td></tr>')
    output_port_html = ""
    if "output_ports" in y:
        for p in y["output_ports"]:
            output_port_html += (
                '<tr><td align=left style=\"padding:5px 0px 5px 0px\">'
                f'&rarr; {p}</td></tr>')
    # Note: keeping this on a single line avoids having to handle comment line
    # markers (e.g. * or ///)
    html = (
        f'<table align=center cellpadding=0 cellspacing=0><tr align=center>'
        f'<td style=\"vertical-align:middle\">'
        f'<table cellspacing=0 cellpadding=0>{input_port_html}</table>'
        f'</td>'
        f'<td align=center style=\"border:solid;padding-left:20px;'
        f'padding-right:20px;vertical-align:middle\" bgcolor=#F0F0F0>'
        f'{y["name"]}</td>'
        f'<td style=\"vertical-align:middle\">'
        f'<table cellspacing=0 cellpadding=0>{output_port_html}</table>'
        f'</td></tr></table>')

    return html


def system_yaml_to_pydrake_system_rst_directive(system_yaml):
    """
    Converts a raw YAML description of a system into a corresponding reST
    directive.
    """
    content = indent(system_yaml.strip(), '    ')
    return f".. pydrake_system::\n\n{content}"


def strip_cpp_comment_cruft(comment):
    """
    Intended to process the text *between* (and not including) the @system and
    @endsystem tags.
    This normalizes against C++ comments as they are seen by `mkdoc`
    (`clang.cindex`).
    - /// and leading * comments markings are simply removed, as we expect the
    tag to reside in a comment block.
    - // comments imply that the remainder of the line should be removed.
    """
    comment = re.sub(r'///', '', comment)
    comment = re.sub(r'\s*//.*', '', comment)
    comment = re.sub(r'^\s*\*', '', comment, flags=re.MULTILINE)
    return comment


def _process_doxygen(s, transform_func):
    """Given a multiline string s (potentially the entire contents of a c++
    file), this finds the @system / @endsystem tags and calls transform_func()
    on their contents.

    Raises exceptions on obsolete (brace-delimited) syntax, missing @endsystem
    tag, and illegal nesting of @system / @endsystem pairs.

    """
    obsolete_syntax = "@system{"
    if s.find(obsolete_syntax) >= 0:
        raise RuntimeError(
            f'obsolete syntax "{obsolete_syntax}" found; use @system /'
            f' @endsystem instead. Found in """\n{s}"""')
    index = 0
    start_tag = "@system"
    end_tag = "@endsystem"
    while s.find(start_tag, index) > 0:
        start = s.find(start_tag, index)
        end = s.find(end_tag, start + len(start_tag))
        if end < 0:
            raise RuntimeError(
                f'missing @endsystem tag in """\n{s[start:]}"""')
        comment = s[start + len(start_tag):end]
        nested = comment.find(start_tag)
        if nested >= 0:
            raise RuntimeError(
                f'illegal nested @system tag in """\n{comment}"""')
        comment_stripped = strip_cpp_comment_cruft(comment)
        transformed = transform_func(comment_stripped)
        s = s[:start] + transformed + s[end + len(end_tag):]
        index = start + len(transformed)
    return s


def process_doxygen_to_sphinx(s):
    """
    Takes a Doxygen C++ comment (as provided by mkdoc directly)
    and replaces any @system / @endsystem tags with a reformatted
    `pydrake_system` reST directive.

    This directive will then be translated by the `PydrakeSystemDirective`
    class in `pydrake_sphinx_extension.py`.
    """
    return _process_doxygen(s, system_yaml_to_pydrake_system_rst_directive)


def process_doxygen_system_tags(s):
    """
    Takes a Doxygen C++ comment (as provided by Doxygen via command-line
    invocation) and replaces any @system / @endsystem tags with reformatted
    HTML, to be viewed by web browsers.
    """
    return _process_doxygen(s, system_yaml_to_html)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError(f"Wrong number of arguments.\n{__doc__}")

    # Load the file in batch into a string.
    with open(sys.argv[1], 'r', encoding='utf-8') as f:
        s = f.read()

    s = process_doxygen_system_tags(s)

    # Print the entire file to stdout
    print(s)
