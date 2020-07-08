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


def generate_system_html(comment):
    """
    Converts a yaml description of a system into an html table.  It is designed to accept the text *between* (and not including) the @system and @endsystem tags.  It allows C++ comments:
      - /// and leading * comments markings are simply removed, as we expect the tag to reside in a comment block.
      - // comments imply that the remainder of the line should be removed.
    """
    comment = re.sub(r'\/\/\/', '', comment)
    comment = re.sub(r'\/\/.*', '', comment)
    comment = re.sub(r'\n\s*\*', '\n', comment)
    try:
        y = yaml.load(comment, Loader=yaml.SafeLoader)
    except yaml.scanner.ScannerError as e:
        print(comment, file=sys.stderr)
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


def process_doxygen_system_tags(s, add_rst_preamble=False):
    """
    Given a multiline string s (potentially the entire contents of a c++ file), this finds the @system / @endsystem tags and calls generate_system_html() on their contents.
    """ 
    index = 0
    start_tag = "@system"
    end_tag = "@endsystem"
    while s.find(start_tag, index) > 0:
        start = s.find(start_tag, index)
        end = s.find(end_tag, start + len(start_tag))
        html = generate_system_html(s[start + len(start_tag):end])
        if add_rst_preamble:
            html = ".. raw:: html\n\n   " + html + "\n\n"
        s = s[:start] + html + s[end + len(end_tag):]
        index = start + len(html)
    return s


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError("Usage: system_doxygen.py input_file")

    # Load the file in batch into a string.
    with open(sys.argv[1], 'r') as f:
        s = f.read()

    s = process_doxygen_system_tags(s)

    # Print the entire file to stdout
    print(s)
