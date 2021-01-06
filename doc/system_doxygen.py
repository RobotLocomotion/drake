#!/usr/bin/env python3

# Usage: 
#   system_doxygen input-file
#
# Will load input-file from disk, replace our custom doxygen @system/@endsystem
# tags into html code that can be rendered in the browser, and print the updated
# file contents to stdout.
#
# This is intended to be used as an INPUT_FILTER for doxygen.
#
# These utilities are also used by mkdoc.py to handle this custom tag in the 
# python documentation.

import re
import sys
import yaml
from textwrap import indent


def system_yaml_to_html(system_yaml):
    """
    Converts a yaml description of a system into an html table.
    """
    try:
        y = yaml.load(system_yaml, Loader=yaml.SafeLoader)
    except yaml.scanner.ScannerError as e:
        print(system_yaml, file=sys.stderr)
        raise(e)

    input_port_html = ""
    if "input_ports" in y:
        for p in y["input_ports"]:
            input_port_html += '<tr><td align=right style=\"padding:5px 0px 5px 0px\">{}&rarr;</td></tr>'.format(p)  # noqa
    output_port_html = ""
    if "output_ports" in y:
        for p in y["output_ports"]:
            output_port_html += '<tr><td align=left style=\"padding:5px 0px 5px 0px\">&rarr; {}</td></tr>'.format(p)  # noqa
    # Note: keeping this on a single line avoids having to handle comment line
    # markers (e.g. * or ///)
    html = '<table align=center cellpadding=0 cellspacing=0><tr align=center><td style=\"vertical-align:middle\"><table cellspacing=0 cellpadding=0>{input_ports}</table></td><td align=center style=\"border:solid;padding-left:20px;padding-right:20px;vertical-align:middle\" bgcolor=#F0F0F0>{name}</td><td style=\"vertical-align:middle\"><table cellspacing=0 cellpadding=0>{output_ports}</table></td></tr></table>'.format(  # noqa
        name=y["name"],
        input_ports=input_port_html,
        output_ports=output_port_html)

    return html


def system_yaml_to_rst_directive(system_yaml):
    """
    Converts a raw yaml description of a system into a corresponding reST directive.
    """
    content = indent(system_yaml.strip(), '    ')
    return f".. pydrake_system::\n\n{content}"


def _strip_cpp_comment_cruft(comment):
    """
    Intended to process the text *between* (and not including) the @system and
    @endsystem tags.
    This normalizes against C++ comments:
    - /// and leading * comments markings are simply removed, as we expect the
    tag to reside in a comment block.
    - // comments imply that the remainder of the line should be removed.
    """
    comment = re.sub(r'\/\/\/', '', comment)
    comment = re.sub(r'\/\/.*', '', comment)
    comment = re.sub(r'\n\s*\*', '\n', comment)
    return comment


def _process_doxygen(s, transform_func):
    """
    Given a multiline string s (potentially the entire contents of a c++ file), this finds the @system / @endsystem tags and calls system_yaml_to_html() on their contents.
    """ 
    index = 0
    start_tag = "@system"
    end_tag = "@endsystem"
    while s.find(start_tag, index) > 0:
        start = s.find(start_tag, index)
        end = s.find(end_tag, start + len(start_tag))
        comment = s[start + len(start_tag):end]
        system_yaml = _strip_cpp_comment_cruft(comment)
        transformed = transform_func(system_yaml)
        s = s[:start] + transformed + s[end + len(end_tag):]
        index = start + len(transformed)
    return s


def process_doxygen_to_sphinx(s):
    return _process_doxygen(s, system_yaml_to_rst_directive)


def process_doxygen_system_tags(s):
    return _process_doxygen(s, system_yaml_to_html)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError("Usage: system_doxygen.py input_file")

    # Load the file in batch into a string.
    with open(sys.argv[1], 'r') as f:
        s = f.read()

    s = process_doxygen_system_tags(s)

    # Print the entire file to stdout
    print(s)
