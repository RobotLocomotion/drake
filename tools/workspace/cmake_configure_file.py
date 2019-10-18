"""A re-implementation of CMake's configure_file substitution semantics.  This
implementation is incomplete, and may not produce the same result as CMake in
all (or even many) cases.

The CMake documentation of the configure_file macro is:
https://cmake.org/cmake/help/latest/command/configure_file.html

"""

import argparse
import os
import re
import sys

from collections import OrderedDict

# Looks like "#cmakedefine VAR ..." or "#cmakedefine01 VAR".
_cmakedefine = re.compile(r'^(\s*)#cmakedefine(01)? ([^ \r\n]+)(.*?)([\r\n]+)')

# Looks like "@VAR@" or "${VAR}".
_varsubst = re.compile(r'^(.*?)(@[^ ]+?@|\$\{[^ ]+?\})(.*)([\r\n]*)')


# Transform a source code line per CMake's configure_file semantics.
#
# The 'definitions' provides values for CMake variables.  The dict's keys are
# the variable names to substitute, and the dict's values are the values to
# substitute.  (The values can be None, for known-but-undefined variable keys.)
#
# The configuration semantics are as follows:
#
# - An input line 'cmakedefine VAR' turns into '#define VAR VALUE' if and only
#   if the 'definitions' dict has a non-None value VALUE for VAR, otherwise it
#   turns into '/* #undef VAR */'.
#
# - An input line 'cmakedefine01 VAR' turns into '#define VAR 1' if and only if
#   the 'definitions' dict has a non-None value for VAR, otherwise it turns
#   into '#define VAR 0'.
#
# - An input line with a substitution '@VAR@' or '${VAR}' replaces the
#   substitution token with the value in 'definitions' dict for that VAR, or
#   else the empty string if the value is None.  It is an error if there is no
#   such key in the dict.
def _transform(line, definitions):
    # Replace define statements.
    match = _cmakedefine.match(line)
    if match:
        blank, maybe01, var, rest, newline = match.groups()
        defined = definitions.get(var) is not None
        if maybe01:
            return blank + '#define ' + var + [' 0', ' 1'][defined] + newline
        elif defined:
            line = blank + '#define ' + var + rest + newline
        else:
            return blank + '/* #undef ' + var + ' */' + newline

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
            assert len(xvarx) >= 3
            assert xvarx[-1] == '@'
            var = xvarx[1:-1]
        assert len(var) > 0

        if var not in definitions:
            raise KeyError('Missing definition for ' + var)
        value = definitions.get(var)
        if value is None:
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
        value = _transform(value, prior_definitions)
    except KeyError:
        return dict()
    if value.startswith('"'):
        assert value.endswith('"')
        value = value[1:-1]
    return {var: value}


# Load our definitions dict, given the command-line args:
# - A command-line '-Dfoo' will add ('foo', 1) to the result.
# - A command-line '-Dfoo=bar' will add ('foo', 'bar') to the result.
# - A command-line '-Ufoo' will add ('foo', None) to the result.
def _setup_definitions(args):
    result = OrderedDict()
    for item in args.defines:
        if '=' in item:
            key, value = item.split('=', 1)
            result[key] = value
        else:
            result[item] = 1

    for item in args.undefines:
        result[item] = None

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
        '-U', metavar='NAME', dest='undefines', action='append', default=[])
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
