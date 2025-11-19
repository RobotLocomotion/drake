"""A re-implementation of CMake's configure_file substitution semantics.  This
implementation is incomplete, and may not produce the same result as CMake in
all (or even many) cases.

The CMake documentation of the configure_file macro is:
https://cmake.org/cmake/help/latest/command/configure_file.html

"""

import argparse
from collections import OrderedDict
import os
import re

# Looks like "#cmakedefine VAR ..." or "#cmakedefine01 VAR".
_cmakedefine = re.compile(
    r"^(\s*)#(\s*)cmakedefine(01)? ([^ \r\n]+)(.*?)([\r\n]+)"
)

# Looks like "${VAR}".
_varsubst = re.compile(r"^(.*)\$\{([^} ]+?)\}(.*)([\r\n]*)")

# Looks like "@VAR@".
_atvarsubst = re.compile(r"^(.*)@([^@ ]+?)@(.*)([\r\n]*)")


# Transform substitutions in a source line according to a specified pattern.
#
# The 'definitions' provides values for CMake variables.  The dict's keys are
# the variable names to substitute, and the dict's values are the values to
# substitute.  (The values can be None, for known-but-undefined variable keys.)
#
# This is used to transform exactly ONE of '@VAR@' or '${VAR}', depending on
# which pattern is specified.
def _transform_substitions(*, line, definitions, used_vars, pattern):
    while (match := pattern.match(line)) is not None:
        before, var, after, newline = match.groups()
        assert len(var) > 0

        value = definitions[var] or ""
        line = before + value + after + newline
        used_vars.add(var)

    return line, used_vars


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
#   turns into '/* #undef VAR */'.  When in strict mode,  it is an error if
#   there is no such key in the dict.
#
# - An input line 'cmakedefine01 VAR' turns into '#define VAR 1' if and only if
#   the 'definitions' dict has a non-None value for VAR, otherwise it turns
#   into '#define VAR 0'.  When in strict mode,  it is an error if there is no
#   such key in the dict.
#
# - An input line with a substitution '@VAR@' or '${VAR}' replaces the
#   substitution token with the value in 'definitions' dict for that VAR, or
#   else the empty string if the value is None.  It is an error if there is no
#   such key in the dict.
def _transform_cmake(*, line, definitions, strict, atonly):
    used_vars = set()

    # Replace define statements.
    match = _cmakedefine.match(line)
    if match:
        blank, _, maybe01, var, rest, newline = match.groups()
        if var not in definitions:
            defined = False
            if strict:
                raise KeyError(var)
        else:
            defined = definitions[var] is not None
            used_vars.add(var)
        if maybe01:
            line = blank + "#define " + var + [" 0", " 1"][defined] + newline
            return line, used_vars
        elif defined:
            line = blank + "#define " + var + rest + newline
        else:
            line = blank + "/* #undef " + var + " */" + newline
            return line, used_vars

    # Replace variable substitutions.
    if not atonly:
        line, used_vars = _transform_substitions(
            line=line,
            definitions=definitions,
            used_vars=used_vars,
            pattern=_varsubst,
        )

    line, used_vars = _transform_substitions(
        line=line,
        definitions=definitions,
        used_vars=used_vars,
        pattern=_atvarsubst,
    )

    return line, used_vars


# Looks like "#undef VAR".
_autoconf_undef = re.compile(r"^(\s*)#undef +([^ \r\n]+)([\r\n]+)")


# Transform a source code line using autoconf format.
# The 'definitions' provides variable values, just like _transform_cmake above.
def _transform_autoconf(*, line, definitions, strict, atonly):
    # 'atonly' isn't meaningful to _transform_autoconf, but the argument is
    # needed in order to have the same signature as _transform_cmake.
    assert not atonly

    used_vars = set()
    match = _autoconf_undef.match(line)
    if match:
        blank, var, newline = match.groups()
        if var in definitions:
            used_vars.add(var)
            value = definitions[var]
            if value is not None:
                line = blank + f"#define {var} {value}" + newline
            else:
                line = blank + f"/* undef {var} */" + newline
        elif strict:
            raise KeyError(
                f"Missing define or undefine decision for {var}"
                " when running in strict=True mode"
            )
        else:
            line = blank + f"/* missing {var} */" + newline
    return line, used_vars


# Looks like "set(VAR value)", maybe with an end-of-line comment.
_set_var = re.compile(r"^\s*set\s*\(\s*(.+)\s+(.+)\s*\)\s*(#.*)?$")


# From a line of CMakeLists.txt, return a set(...) key-value pair, if found.
def _extract_definition(line, prior_definitions):
    match = _set_var.match(line)
    if not match:
        return dict()
    var, value, _ = match.groups()
    try:
        value, _ = _transform_cmake(
            line=value,
            definitions=prior_definitions,
            strict=False,
            atonly=False,
        )
    except KeyError:
        return dict()
    if value.startswith('"'):
        assert value.endswith('"')
        value = value[1:-1]
    return {var: value}


# Load our definitions dict, given the command-line args:
# - A command-line '-Dfoo' will add ('foo', '1') to the result.
# - A command-line '-Dfoo=bar' will add ('foo', 'bar') to the result.
# - A command-line '-Ufoo' will add ('foo', None) to the result.
def _setup_definitions(args):
    result = OrderedDict()
    for item in args.defines:
        if "=" in item:
            key, value = item.split("=", 1)
            result[key] = value
        else:
            result[item] = "1"

    for item in args.undefines:
        result[item] = None

    cmakelist_keys = set()
    for filename in args.cmakelists:
        with open(filename, "r") as cmakelist:
            for line in cmakelist.readlines():
                definition = _extract_definition(line, result)
                result.update(definition)
                cmakelist_keys.update(definition.keys())

    return result, cmakelist_keys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", metavar="FILE", action="append", default=[])
    parser.add_argument("--output", metavar="FILE", action="append", default=[])
    parser.add_argument(
        "-D", metavar="NAME", dest="defines", action="append", default=[]
    )
    parser.add_argument(
        "-U", metavar="NAME", dest="undefines", action="append", default=[]
    )
    parser.add_argument("--cmakelists", action="append", default=[])
    parser.add_argument("--strict", action="store_true")
    modifiers = parser.add_mutually_exclusive_group()
    modifiers.add_argument(
        "--autoconf",
        action="store_true",
        help="The input file is in autoconf format, not cmake format.",
    )
    modifiers.add_argument("--atonly", action="store_true")
    args = parser.parse_args()
    if len(args.input) == 0:
        parser.error("There must be at least one --input")
    if len(args.input) != len(args.output):
        parser.error("The number of --input and --output must be congruent")
    definitions, cmakelist_keys = _setup_definitions(args)

    transformer = _transform_autoconf if args.autoconf else _transform_cmake
    total_used_vars = set()
    missing_vars = set()
    for input_path, output_path in zip(args.input, args.output):
        with open(input_path, "r") as input_file:
            with open(output_path + ".tmp", "w") as output_file:
                for input_line in input_file.readlines():
                    try:
                        output_line, used_vars = transformer(
                            line=input_line,
                            definitions=definitions,
                            strict=args.strict,
                            atonly=args.atonly,
                        )
                        output_file.write(output_line)
                        total_used_vars |= used_vars
                    except KeyError as e:
                        missing_vars.add(e.args[0])
    if missing_vars:
        raise RuntimeError(
            f"The definitions of {sorted(missing_vars)} were"
            " required, but missing."
        )
    unused_vars = definitions.keys() - cmakelist_keys - total_used_vars
    if unused_vars:
        raise RuntimeError(
            f"The definitions of {sorted(unused_vars)} were"
            " ignored and therefore seem like dead code;"
            " remove them from defines= or undefines=."
        )
    for output_path in args.output:
        os.rename(output_path + ".tmp", output_path)


if __name__ == "__main__":
    main()
