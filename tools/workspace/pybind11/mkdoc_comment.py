# -*- coding: utf-8 -*-
#
#  Derived from https://github.com/pybind/pybind11/
#
#  Copyright (c) 2016 Wenzel Jakob <wenzel.jakob@epfl.ch>,
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from this
#     software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
#   Converts Doxygen-formatted string to look presentable in a Python
#   docstring.
#

import re
import textwrap

from drake.doc.doxygen_cxx.system_doxygen import process_doxygen_to_sphinx


def process_comment(comment):
    # Remove C++ comment syntax
    s = remove_cpp_comment_syntax(comment)

    # Remove HTML comments. Must occur before Doxygen commands are parsed
    # since they may be used to workaround limitations related to line breaks
    # in Doxygen.
    # TODO (betsymcphail): Not tested
    s = remove_html_comments(s)

    # Markdown to reStructuredText.
    s = markdown_to_restructuredtext(s)

    # HTML tags. Support both lowercase and uppercase tags.
    # TODO (betsymcphail): Not tested
    s = replace_html_tags(s)

    # TODO (betsymcphail): Not tested
    s = s.replace('``true``', '``True``')
    s = s.replace('``false``', '``False``')

    # Exceptions
    s = replace_exceptions(s)

    s = process_doxygen_commands(s)

    # Reflow text where appropriate.
    s = reflow(s)

    return s


def remove_cpp_comment_syntax(s):
    result = ''
    leading_spaces = float('inf')
    for line in s.expandtabs(tabsize=4).splitlines():
        line = line.strip()
        if line.startswith('/*!'):
            line = line[3:]
        if line.startswith('/*'):
            line = line[2:].lstrip('*')
        if line.endswith('*/'):
            line = line[:-2].rstrip('*')
        # http://www.doxygen.nl/manual/docblocks.html#memberdoc
        if line.startswith('///<'):
            line = line[4:]
        if line.startswith('///') or line.startswith('//!'):
            line = line[3:]
        if line.startswith('*'):
            line = line[1:]
        if len(line) > 0:
            leading_spaces = min(leading_spaces,
                                 len(line) - len(line.lstrip()))
        result += line + '\n'

    if leading_spaces != float('inf'):
        result2 = ""
        for line in result.splitlines():
            result2 += line[leading_spaces:] + '\n'
        result = result2

    return result


def remove_html_comments(s):
    return re.sub(r'<!--(.*?)-->', r'', s, flags=re.DOTALL)


def markdown_to_restructuredtext(s):
    # TODO(jamiesnape): Find a third-party library do this?
    # Convert _italics_ to *italics*.
    s = re.sub(r'([\s\-,;:!.()]+)_([^\s_]+|[^\s_].*?[^\s_])_([\s\-,;:!.()]+)',
               r'\1*\2*\3', s, flags=re.DOTALL)
    # Convert __bold__ to **bold**.
    s = re.sub(
        r'([\s\-,;:!.()]+)__([^\s_]+|[^\s_].*?[^\s_])__([\s\-,;:!.()]+)',
        r'\1**\2**\3', s, flags=re.DOTALL)
    # Convert `typewriter` to ``typewriter``.
    s = re.sub(r'([\s\-,;:!.()]+)`([^\s`]|[^\s`].*?[^\s`])`([\s\-,;:!.()]+)',
               r'\1``\2``\3', s, flags=re.DOTALL)
    # Convert [Link](https://example.org) to `Link <https://example.org>`_.
    s = re.sub(r'\[(.*?)\]\(([\w:.?/#]+)\)', r'`\1 <\2>`_', s,
               flags=re.DOTALL)

    s = re.sub(r'#### (.*?) ####', r'\n*\1*\n', s)
    s = re.sub(r'#### (.*?)', r'\n*\1*\n', s)
    s = re.sub(r'### (.*?) ###', r'\n**\1**\n', s)
    s = re.sub(r'### (.*?)', r'\n**\1**\n', s)

    s = replace_with_header(r'## (.*?) ##', '-', s)
    s = replace_with_header(r'## (.*?)', '-', s)
    s = replace_with_header(r'# (.*?) #', '=', s)
    s = replace_with_header(r'# (.*?)', '=', s)
    return s


def replace_with_header(pattern, token, s, **kwargs):
    def repl(match):
        return '\n{}\n{}\n'.format(match.group(1),
                                   token * len(match.group(1)))
    return re.sub(pattern, repl, s, **kwargs)


def replace_html_tags(s):
    s = re.sub(r'<tt>(.*?)</tt>', r'``\1``', s,
               flags=re.DOTALL | re.IGNORECASE)
    s = re.sub(r'<pre>(.*?)</pre>', r"```\n\1\n```\n", s,
               flags=re.DOTALL | re.IGNORECASE)
    s = re.sub(r'<em>(.*?)</em>', r'*\1*', s, flags=re.DOTALL | re.IGNORECASE)
    s = re.sub(r'<b>(.*?)</b>', r'**\1**', s, flags=re.DOTALL | re.IGNORECASE)

    s = re.sub(r'<li>', r'\n\n* ', s, flags=re.IGNORECASE)
    s = re.sub(r'</?ol( start=[0-9]+)?>', r'', s, flags=re.IGNORECASE)
    s = re.sub(r'</?ul>', r'', s, flags=re.IGNORECASE)
    s = re.sub(r'</li>', r'\n\n', s, flags=re.IGNORECASE)

    s = re.sub(r'<a href="([\w:.?/#]+)">(.*?)</a>', r'`\2 <\1>`_', s,
               flags=re.DOTALL | re.IGNORECASE)

    s = re.sub(r'<br/?>', r'\n\n', s, flags=re.IGNORECASE)

    s = replace_with_header(r'<h1>(.*?)</h1>', '=', s, flags=re.IGNORECASE)
    s = replace_with_header(r'<h2>(.*?)</h2>', '-', s, flags=re.IGNORECASE)
    s = re.sub(r'<h3>(.*?)</h3>', r'\n**\1**\n', s, flags=re.IGNORECASE)
    s = re.sub(r'<h4>(.*?)</h4>', r'\n*\1*\n', s, flags=re.IGNORECASE)
    return s


def replace_exceptions(s):
    s = s.replace('std::bad_alloc', 'MemoryError')
    s = s.replace('std::bad_any_cast', 'RuntimeError')
    s = s.replace('std::bad_array_new_length', 'MemoryError')
    s = s.replace('std::bad_cast', 'RuntimeError')
    s = s.replace('std::bad_exception', 'RuntimeError')
    s = s.replace('std::bad_function_call', 'RuntimeError')
    s = s.replace('std::bad_optional_access', 'RuntimeError')
    s = s.replace('std::bad_typeid', 'RuntimeError')
    s = s.replace('std::bad_variant_access', 'RuntimeError')
    s = s.replace('std::bad_weak_ptr', 'RuntimeError')
    s = s.replace('std::domain_error', 'ValueError')
    s = s.replace('std::exception', 'RuntimeError')
    s = s.replace('std::future_error', 'RuntimeError')
    s = s.replace('std::invalid_argument', 'ValueError')
    s = s.replace('std::length_error', 'ValueError')
    s = s.replace('std::logic_error', 'RuntimeError')
    s = s.replace('std::out_of_range', 'ValueError')
    s = s.replace('std::overflow_error', 'RuntimeError')
    s = s.replace('std::range_error', 'ValueError')
    s = s.replace('std::regex_error', 'RuntimeError')
    s = s.replace('std::runtime_error', 'RuntimeError')
    s = s.replace('std::system_error', 'RuntimeError')
    s = s.replace('std::underflow_error', 'RuntimeError')
    return s


def process_doxygen_commands(s):
    # Doxygen tags
    cpp_group = r'([\w:*()]+)'
    param_group = r'([\[\w,\]]+)'

    s = re.sub(r'[@\\][cp]\s+%s' % cpp_group, r'``\1``', s)
    s = re.sub(r'[@\\](?:a|e|em)\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]b\s+%s' % cpp_group, r'**\1**', s)
    s = re.sub(r'[@\\]param%s?\s+%s' % (param_group, cpp_group),
               r'\n\n$Parameter ``\2``:\n\n', s)
    s = re.sub(r'[@\\]tparam%s?\s+%s' % (param_group, cpp_group),
               r'\n\n$Template parameter ``\2``:\n\n', s)
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\]retval\s+%s' % cpp_group,
               r'\n\n$Returns ``\1``:\n\n', s)

    # Ordering is significant for command names with a common prefix.
    for in_, out_ in (
        ('result', 'Returns'),
        ('returns', 'Returns'),
        ('return', 'Returns'),
        ('attention', 'Attention'),
        ('authors', 'Authors'),
        ('author', 'Authors'),
        ('bug', 'Bug report'),
        ('copyright', 'Copyright'),
        ('date', 'Date'),
        ('deprecated', 'Deprecated'),
        ('exception', 'Raises'),
        ('invariant', 'Invariant'),
        ('note', 'Note'),
        ('post', 'Postcondition'),
        ('pre', 'Precondition'),
        ('remarks', 'Remark'),
        ('remark', 'Remark'),
        ('sa', 'See also'),
        ('see', 'See also'),
        ('since', 'Since'),
        ('extends', 'Extends'),
        ('throws', 'Raises'),
        ('throw', 'Raises'),
        ('test', 'Test case'),
        ('todo', 'Todo'),
        ('version', 'Version'),
        ('warning', 'Warning'),
    ):
        s = re.sub(r'[@\\]%s\s*' % in_, r'\n\n$%s:\n\n' % out_, s)

    s = re.sub(r'[@\\]details\s*', r'\n\n', s)
    s = re.sub(r'[@\\](?:brief|short)\s*', r'', s)
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\]ref\s+', r'', s)

    for start_, end_ in (
        ('code', 'endcode'),
        ('verbatim', 'endverbatim')
    ):
        s = re.sub(r'[@\\]%s(?:\{\.\w+\})?\s?(.*?)\s?[@\\]%s' % (start_, end_),
                   r"```\n\1\n```\n", s, flags=re.DOTALL)

    s = process_doxygen_to_sphinx(s)

    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\](?:end)?htmlonly\s+', r'', s)

    # These commands are always prefixed with an @ sign.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'@[{}]\s*', r'', s)

    # Doxygen list commands.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\](?:arg|li)\s+', r'\n\n* ', s)

    # Doxygen sectioning commands.
    s = replace_with_header(r'[@\\]section\s+\w+\s+(.*)', '=', s)
    s = replace_with_header(r'[@\\]subsection\s+\w+\s+(.*)', '-', s)
    s = re.sub(r'[@\\]subsubsection\s+\w+\s+(.*)', r'\n**\1**\n', s)

    # Doxygen LaTeX commands.
    s = re.sub(r'[@\\]f\$\s*(.*?)\s*[@\\]f\$', r':math:`\1`', s,
               flags=re.DOTALL)
    s = re.sub(r'[@\\]f\[\s*(.*?)\s*[@\\]f\]', r'\n\n.. math:: \1\n\n', s,
               flags=re.DOTALL)
    s = re.sub(r'[@\\]f\{([\w*]+)\}\s*(.*?)\s*[@\\]f\}',
               r'\n\n.. math:: \\begin{\1}\2\\end{\1}\n\n', s, flags=re.DOTALL)

    # Drake-specific Doxygen aliases.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\]default\s+', r'\n$*Default:* ', s)

    # Omit tparam scalar type boilerplate; the python documentation already
    # presents this information in a more useful manner.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\]tparam_default_scalar\s+', r'', s)
    s = re.sub(r'[@\\]tparam_nonsymbolic_scalar\s+', r'', s)
    s = re.sub(r'[@\\]tparam_double_only\s+', r'', s)

    # Remove these commands that take no argument. Ordering is significant for
    # command names with a common prefix.
    for cmd_ in (
        '~english',
        '~',
        'callergraph',
        'callgraph',
        'hidecallergraph',
        'hidecallgraph',
        'hideinitializer',
        'nosubgrouping',
        'privatesection',
        'private',
        'protectedsection',
        'protected',
        'publicsection',
        'public',
        'pure',
        'showinitializer',
        'static',
        'tableofcontents',
    ):
        s = re.sub(r'[@\\]%s\s+' % cmd_, r'', s)

    # Remove these commands and their one optional single-word argument.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        'dir',
        'file',
    ]:
        s = re.sub(r'[@\\]%s( +[\w:./]+)?\s+' % cmd_, r'', s)

    # Remove these commands and their one optional single-line argument.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        'mainpage',
        'name'
        'overload',
    ]:
        s = re.sub(r'[@\\]%s( +.*)?\s+' % cmd_, r'', s)

    # Remove these commands and their one single-word argument. Ordering is
    # significant for command names with a common prefix.
    for cmd_ in [
        'anchor',
        'copybrief',
        'copydetails',
        'copydoc',
        'def',
        'dontinclude',
        'enum',
        'example',
        'extends',
        'htmlinclude',
        'idlexcept',
        'implements',
        'includedoc',
        'includelineno',
        'include',
        'latexinclude',
        'memberof',
        'namespace',
        'package',
        'relatedalso',
        'related',
        'relatesalso',
        'relates',
        'verbinclude',
    ]:
        s = re.sub(r'[@\\]%s\s+[\w:.]+\s+' % cmd_, r'', s)

    # Remove these commands and their one single-line argument. Ordering is
    # significant for command names with a common prefix.
    for cmd_ in [
        'addindex',
        'fn',
        'ingroup',
        'line',
        'property',
        'skipline',
        'skip',
        'typedef',
        'until',
        'var',
    ]:
        s = re.sub(r'[@\\]%s\s+.*\s+' % cmd_, r'', s)

    # Remove this command and its one single-word argument and one
    # optional single-word argument.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\]headerfile\s+[\w:.]+( +[\w:.]+)?\s+', r'', s)

    # Remove these commands and their one single-word argument and one
    # optional single-line argument.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        'addtogroup',
        'weakgroup',
    ]:
        s = re.sub(r'[@\\]%s\s+[\w:.]( +.*)?\s+' % cmd_, r'', s)

    # Remove these commands and their one single-word argument and one
    # single-line argument. Ordering is significant for command names with a
    # common prefix.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        'snippetdoc',
        'snippetlineno',
        'snippet',
    ]:
        s = re.sub(r'[@\\]%s\s+[\w:.]\s+.*\s+' % cmd_, r'', s)

    # Remove these commands and their one single-word argument and two
    # optional single-word arguments.
    for cmd_ in [
        'category',
        'class',
        'interface',
        'protocol',
        'struct',
        'union',
    ]:
        s = re.sub(r'[@\\]%s\s+[\w:.]+( +[\w:.]+){0,2}\s+' % cmd_, r'', s)

    # Remove these commands and their one single-word argument, one optional
    # quoted argument, and one optional single-word arguments.
    # TODO (betsymcphail): Not tested
    for cmd_ in [
        'diafile',
        'dotfile',
        'mscfile',
    ]:
        s = re.sub(
           r'[@\\]%s\s+[\w:.]+(\s+".*?")?(\s+[\w:.]+=[\w:.]+)?s+' % cmd_,
           r'', s)

    # Remove these pairs of commands and any text in between.
    for start_, end_ in (
        ('cond', 'endcond'),
        ('docbookonly', 'enddocbookonly'),
        ('dot', 'enddot'),
        ('internal', 'endinternal'),
        ('latexonly', 'endlatexonly'),
        ('manonly', 'endmanonly'),
        ('msc', 'endmsc'),
        ('rtfonly', 'endrtfonly'),
        ('secreflist', 'endsecreflist'),
        ('startuml', 'enduml'),
        ('xmlonly', 'endxmlonly'),
    ):
        s = re.sub(r'[@\\]%s\s?(.*?)\s?[@\\]%s' % (start_, end_), r'', s,
                   flags=re.DOTALL)

        # Some command pairs may bridge multiple comment blocks, so individual
        # start and end commands may appear alone.
        s = re.sub(r'[@\\]%s\s+' % start_, r'', s)
        s = re.sub(r'[@\\]%s\s+' % end_, r'', s)

    # Remove auto-linking character. Be sure to remove only leading % signs.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'(\s+)%(\S+)', r'\1\2', s)

    # Doxygen escaped characters.
    # TODO (betsymcphail): Not tested
    s = re.sub(r'[@\\]n\s+', r'\n\n', s)

    # Ordering of ---, --, @, and \ is significant.
    # TODO (betsymcphail): Not tested
    for escaped_ in (
        '---',
        '--',
        '::',
        r'\.',
        '"',
        '&',
        '#',
        '%',
        '<',
        '>',
        r'\$',
        '@',
        r'\\',
    ):
        s = re.sub(r'[@\\](%s)' % escaped_, r'\1', s)

    return s


def reflow(s):
    wrapper = textwrap.TextWrapper()
    wrapper.break_long_words = False
    wrapper.break_on_hyphens = False
    wrapper.drop_whitespace = True
    wrapper.expand_tabs = True
    wrapper.replace_whitespace = True
    wrapper.width = 70
    wrapper.initial_indent = wrapper.subsequent_indent = ''

    result = ''
    in_code_segment = False
    for x in re.split(r'(```)', s):
        if x == '```':
            if not in_code_segment:
                result += '\n::\n'
            else:
                result += '\n\n'
            in_code_segment = not in_code_segment
        elif in_code_segment:
            result += '    '.join(('\n' + x.strip()).splitlines(True))
        else:
            in_rst_directive = False
            for y in re.split(r'(?: *\n){2,}', x):
                # Do not reflow rst directives.
                # These appear, for instance, through our implementation of the
                # custom @system / @endsystem tags.
                if in_rst_directive:
                    if y[:3] != '   ':
                        in_rst_directive = False
                        # and program flow continues in this loop.
                    else:
                        result += y + '\n\n'
                        continue
                elif y[:3] == '.. ':
                    in_rst_directive = True
                    result += y + '\n\n'
                    continue
                lines = re.split(r'(?: *\n *)', y)
                # Do not reflow lists or section headings.
                if (re.match(r'^(?:[*+\-]|[0-9]+[.)]) ', lines[0])
                    or (len(lines) > 1
                        and (lines[1] == '=' * len(lines[0])
                             or lines[1] == '-' * len(lines[0])))):
                    result += y + '\n\n'
                else:
                    wrapped = wrapper.fill(re.sub(r'\s+', ' ', y).strip())
                    if len(wrapped) > 0 and wrapped[0] == '$':
                        result += wrapped[1:] + '\n'
                        wrapper.initial_indent = \
                            wrapper.subsequent_indent = ' ' * 4
                    else:
                        if len(wrapped) > 0:
                            result += wrapped + '\n\n'
                        wrapper.initial_indent = wrapper.subsequent_indent = ''
    return result.rstrip().lstrip('\n')
