#!/usr/bin/env python

"""A re-implentation of CMake's configure_file substitution semanatics.  This
implementation is incomplete, and may not produce the same result as CMake in
all (or even many) cases.

"""

import argparse
import logging
import os
import re
import sys

from collections import OrderedDict

# Looks like "#cmakedefine VAR ..." or "#cmakedefine01 VAR".
_cmakedefine = re.compile(r'^#cmakedefine(01)? ([^ \r\n]+)(.*?)([\r\n]*)')

# Looks like "@VAR@" or "@{VAR}".
_varsubst = re.compile(r'^(.*?)(@.+?@|\$\{.+?\})(.*)([\r\n]*)')

# Transform a source code line per CMake's configure_file semantics.
def _transform(line, definitions, raise_on_failure=False):
    # Replace define statements.
    match = _cmakedefine.match(line)
    if match:
        maybe01, var, rest, newline = match.groups()
        defined = var in definitions
        if maybe01:
            return '#define ' + var + [' 0', ' 1'][defined] + newline
        elif not defined:
            return '/* #undef ' + var + ' */' + newline
        else:
            line = '#define ' + var + rest + newline

    # Replace variable substitutions.
    while True:
        match = _varsubst.match(line)
        if not match:
            break
        before, xvarx, after, newline = match.groups()
        if xvarx[0] == '$':
            assert len(xvarx) >= 4
            assert xvarx[1] == '{'
            assert xvarx[-1] == '}'
            var = xvarx[2:-1]
        elif xvarx[0] == '@':
            assert len(xvarx) >= 4
            assert xvarx[-1] == '@'
            var = xvarx[1:-1]
        assert len(var) > 0

        value = definitions.get(var)
        if value is None:
            if raise_on_failure:
                raise KeyError(value)
            logging.warn('Missing definition for ' + var)
            value = ''
        line = before + value + after + newline

    return line

# Looks like "set(VAR value)".
_set_var = re.compile(r'^\s*set\s*\(\s*(.+)\s+(.+)\s*\)\s*$')

# From a line of CMakeLists.txt, return a set(...) key-value pair, if found.
def _extract_definition(line, prior_definitions):
    match = _set_var.match(line)
    if not match:
        return dict()
    var, value = match.groups()
    try:
        value = _transform(value, prior_definitions, raise_on_failure=True)
    except KeyError:
        return dict()
    if value.startswith('"'):
        assert value.endswith('"')
        value = value[1:-1]
    return {var: value}

# Load our definitions dict, given the command-line args.
def _setup_definitions(args):
    result = OrderedDict()
    for item in args.defines:
        if '=' in item:
            key, value = item.split('=', 1)
            result[key] = value
        else:
            result[item] = True

    for filename in args.cmakelists:
        with open(filename, 'r') as cmakelist:
            for line in cmakelist.readlines():
                definition = _extract_definition(line, result)
                result.update(definition)

    return result

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', metavar='FILE')
    parser.add_argument('--output', metavar='FILE')
    parser.add_argument(
        '-D', metavar='NAME', dest='defines', action='append', default=[])
    parser.add_argument(
        '--cmakelists', action='append', default=[])
    args = parser.parse_args()
    if args.input is None or args.output is None:
        parser.print_usage()
        sys.exit(1)
    definitions = _setup_definitions(args)

    with open(args.input, 'r') as input_file:
        with open(args.output + '.tmp', 'w') as output_file:
            for input_line in input_file.readlines():
                output_line = _transform(input_line, definitions)
                output_file.write(output_line)
    os.rename(args.output + '.tmp', args.output)

if __name__ == '__main__':
    main()
