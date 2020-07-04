#!/usr/bin/env python3

# Usage:
#   mk_system_doc input-file
# Will load input-file from disk, replace our custom doxygen @system tags into
# html code that can be rendered in the browser, and print the updated file
# contents to stdout.
#
# This is intended to be used as an INPUT_FILTER for doxygen.
#
# The generate_system_html is also used by mkdoc.py to handle this custom tag
# in the python documentation.

import re
import sys
import yaml


def generate_system_html(comment):
    comment = re.sub(r'\/\/\/', '', comment)
    comment = re.sub(r'\/\/.*', '', comment)
    comment = re.sub(r'\n\s*\*', '\n', comment)
    y = yaml.load(comment, Loader=yaml.SafeLoader)
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
    # Process any @system{ } blocks
    index = 0
    start_tag = "@system"
    end_tag = "@endsystem"
    while s.find(start_tag, index) > 0:
        start = s.find(start_tag, index)
        # Ignore deprecated @system{ } style tag.
        if (s[start+len(start_tag)] == "{"):
            index = start + len(start_tag)
            continue
        end = s.find(end_tag, start + len(start_tag))
        html = generate_system_html(s[start + len(start_tag):end])
        if add_rst_preamble:
            html = ".. raw:: html\n\n   " + html + "\n\n"
        s = s[:start] + html + s[end + len(end_tag):]
        index = start + len(html)
    return s


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise RuntimeError("Usage: mk_system_doc.py input_file")

    # Load the file in batch into a string.
    with open(sys.argv[1], 'r') as f:
        s = f.read()

    s = process_doxygen_system_tags(s)

    # Print the entire file to stdout
    print(s)
