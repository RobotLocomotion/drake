# bzlcodestyle.py - A Skylark code style checker.
#
# Copyright 2017 Robot Locomotion Group @ CSAIL
#
# Portions of this code are based on pycodestyle.py:
#
# Copyright 2006-2009 Johann C. Rocholl <johann@rocholl.net>
# Copyright 2009-2014 Florent Xicluna <florent.xicluna@gmail.com>
# Copyright 2014-2016 Ian Lee <ianlee1521@gmail.com>
#
# Permission is hereby granted, free of charge, to any person
# obtaining a copy of this software and associated documentation files
# (the "Software"), to deal in the Software without restriction,
# including without limitation the rights to use, copy, modify, merge,
# publish, distribute, sublicense, and/or sell copies of the Software,
# and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be
# included in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
# BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
# ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import pycodestyle
import sys


def whitespace_around_named_parameter_equals_positive(logical_line, tokens):
    """Require spaces around the '=' sign in function arguments."""
    # N.B. Nominal `pycodestyle` includes a method named
    # `whitespace_around_named_parameter_equals`, which is the negative case.
    # Python3 struggles with a same-named method because it does not want to
    # order functions (the sorting tuple is `(name, function, args)`).
    # We must avoid colliding with this name, and ensure this takes precedence.
    parens = 0
    no_space = False
    prev_end = None
    annotated_func_arg = False
    in_def = logical_line.startswith('def')
    message = "B251 expected spaces around keyword / parameter equals"
    for token_type, text, start, end, line in tokens:
        if token_type == pycodestyle.tokenize.NL:
            continue
        if no_space:
            no_space = False
            if start == prev_end:
                yield (prev_end, message)
        if token_type == pycodestyle.tokenize.OP:
            if text in '([':
                parens += 1
            elif text in ')]':
                parens -= 1
            elif in_def and text == ':' and parens == 1:
                annotated_func_arg = True
            elif parens and text == ',' and parens == 1:
                annotated_func_arg = False
            elif parens and text == '=' and not annotated_func_arg:
                no_space = True
                if start == prev_end:
                    yield (prev_end, message)
            if not parens:
                annotated_func_arg = False

        prev_end = end


def _main():
    """Parse options and run checks on Skylark source."""

    pycodestyle.register_check(
        whitespace_around_named_parameter_equals_positive)

    style_guide = pycodestyle.StyleGuide(parse_argv=True)
    options = style_guide.options

    ignore = set(options.ignore)
    ignore.add('E251')  # Skylark wants spaces around named parameters
    ignore.add('E711')  # Skylark has no `is`
    ignore.add('E721')  # Skylark has no `isinstance`
    options.ignore = tuple(ignore)

    report = style_guide.check_files()

    if options.statistics:
        report.print_statistics()

    if report.total_errors:
        if options.count:
            sys.stderr.write(str(report.total_errors) + '\n')
        sys.exit(1)


if __name__ == '__main__':
    _main()
