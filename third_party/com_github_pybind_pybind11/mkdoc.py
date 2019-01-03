#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#  Syntax:
#     mkdoc.py [-output=<file>] [-I<path> ..] [-quiet] [.. header files ..]
#
#  Extract documentation from C++ header files to use it in Python bindings
#

from collections import OrderedDict, defaultdict
from fnmatch import fnmatch
import os
import platform
import re
import sys
from tempfile import NamedTemporaryFile, mkdtemp
import textwrap

from clang import cindex
from clang.cindex import AccessSpecifier, CursorKind, TypeKind

CLASS_KINDS = [
    CursorKind.CLASS_DECL,
    CursorKind.STRUCT_DECL,
    CursorKind.CLASS_TEMPLATE,
]

FUNCTION_KINDS = [
    CursorKind.FUNCTION_DECL,
    CursorKind.FUNCTION_TEMPLATE,
    CursorKind.CONVERSION_FUNCTION,
    CursorKind.CXX_METHOD,
    CursorKind.CONSTRUCTOR,
]


RECURSE_LIST = [
    CursorKind.TRANSLATION_UNIT,
    CursorKind.NAMESPACE,
    CursorKind.CLASS_DECL,
    CursorKind.STRUCT_DECL,
    CursorKind.ENUM_DECL,
    CursorKind.CLASS_TEMPLATE
]

PRINT_LIST = CLASS_KINDS + FUNCTION_KINDS + [
    CursorKind.ENUM_DECL,
    CursorKind.ENUM_CONSTANT_DECL,
    CursorKind.FIELD_DECL,
    CursorKind.TYPE_ALIAS_DECL,  # using x = y
    CursorKind.TYPEDEF_DECL
]

CPP_OPERATORS = {
    '<=': 'le', '>=': 'ge', '==': 'eq', '!=': 'ne', '[]': 'array',
    '+=': 'iadd', '-=': 'isub', '*=': 'imul', '/=': 'idiv', '%=':
    'imod', '&=': 'iand', '|=': 'ior', '^=': 'ixor', '<<=': 'ilshift',
    '>>=': 'irshift', '++': 'inc', '--': 'dec', '<<': 'lshift', '>>':
    'rshift', '&&': 'land', '||': 'lor', '!': 'lnot', '~': 'bnot',
    '&': 'band', '|': 'bor', '+': 'add', '-': 'sub', '*': 'mul', '/':
    'div', '%': 'mod', '<': 'lt', '>': 'gt', '=': 'assign', '()': 'call'
}

CPP_OPERATORS = OrderedDict(
    sorted(CPP_OPERATORS.items(), key=lambda t: -len(t[0])))

# 'Broadphase' culling; do not recurse inside these symbols.
SKIP_RECURSE_NAMES = [
    'DRAKE_COPYABLE_DEMAND_COPY_CAN_COMPILE',
    'Eigen',
    'detail',
    'google',
    'internal',
    'std',
    'tinyxml2',
]

# Exceptions to `SKIP_RECURSE_NAMES`; only one degree of exception is made
# (i.e., nested symbols are still subject to `SKIP_RECURSE_NAMES`).
SKIP_RECURSE_EXCEPTIONS = [
    # TODO(eric.cousineau): Remove once #9366 is complete and all deprecated
    # symbols are removed.
    ('drake', 'multibody', 'internal'),
]

# Filter based on partial names.
SKIP_PARTIAL_NAMES = [
    'operator new',
    'operator delete',
    'operator=',
    'operator->',
    'operator<<',
    'operator>>',
]

# Filter based on access.
SKIP_ACCESS = [
    AccessSpecifier.PRIVATE,
]


def utf8(s):
    # Decodes a string to utf8.
    return s.decode('utf8')


class Symbol(object):
    """
    Contains a cursor and additional processed metadata.
    """
    def __init__(self, cursor, name_chain, include, line, comment):
        self.cursor = cursor
        self.name_chain = name_chain
        self.include = include
        self.line = line
        self.comment = comment

    def sorting_key(self):
        return (self.name_chain, self.include, self.line)


def eprint(*args):
    print(*args, file=sys.stderr)


def is_accepted_cursor(cursor, name_chain):
    """
    Determines if a symbol should be visited or not.
    """
    name = utf8(cursor.spelling)
    if name in SKIP_RECURSE_NAMES:
        if tuple(name_chain) not in SKIP_RECURSE_EXCEPTIONS:
            return False
    for bad in SKIP_PARTIAL_NAMES:
        if bad in name:
            return False
    if cursor.access_specifier in SKIP_ACCESS:
        return False
    # TODO(eric.cousineau): Remove `cursor.is_default_method()`? May make
    # things unstable.
    if cursor.kind in CLASS_KINDS and not cursor.is_definition():
        # Don't process forward declarations.  If we did, we'd define the class
        # overview documentation twice; both cursors have a .raw_comment value.
        return False
    return True


def sanitize_name(name):
    """
    Sanitizes a C++ symbol to be variable-friendly.
    """
    name = re.sub(r'type-parameter-0-([0-9]+)', r'T\1', name)
    for k, v in CPP_OPERATORS.items():
        name = name.replace('operator%s' % k, 'operator_%s' % v)
    name = re.sub('<.*>', '', name)
    name = name.replace('::', '_')
    name = ''.join([ch if ch.isalnum() else '_' for ch in name])
    name = re.sub('_+', '_', name)
    return name


# TODO(jamiesnape): Refactor into multiple functions and unit test.
def process_comment(comment):
    """
    Converts Doxygen-formatted string to look presentable in a Python
    docstring.
    """
    result = ''

    # Remove C++ comment syntax
    leading_spaces = float('inf')
    for s in comment.expandtabs(tabsize=4).splitlines():
        s = s.strip()
        if s.startswith('/*!'):
            s = s[3:]
        if s.startswith('/*'):
            s = s[2:].lstrip('*')
        if s.endswith('*/'):
            s = s[:-2].rstrip('*')
        if s.startswith('///') or s.startswith('//!'):
            s = s[3:]
        if s.startswith('*'):
            s = s[1:]
        if len(s) > 0:
            leading_spaces = min(leading_spaces, len(s) - len(s.lstrip()))
        result += s + '\n'

    if leading_spaces != float('inf'):
        result2 = ""
        for s in result.splitlines():
            result2 += s[leading_spaces:] + '\n'
        result = result2

    s = result

    # Remove HTML comments. Must occur before Doxygen commands are parsed
    # since they may be used to workaround limitations related to line breaks
    # in Doxygen.
    s = re.sub(r'<!--(.*?)-->', r'', s, flags=re.DOTALL)

    # Markdown to reStructuredText.
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

    def replace_with_header(pattern, token, s, **kwargs):
        def repl(match):
            return '\n{}\n{}\n'.format(match.group(1),
                                       token * len(match.group(1)))
        return re.sub(pattern, repl, s, **kwargs)

    s = replace_with_header(r'## (.*?) ##', '-', s)
    s = replace_with_header(r'## (.*?)', '-', s)
    s = replace_with_header(r'# (.*?) #', '=', s)
    s = replace_with_header(r'# (.*?)', '=', s)

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
    s = re.sub(r'[@\\]ref\s+', r'', s)

    for start_, end_ in (
        ('code', 'endcode'),
        ('verbatim', 'endverbatim')
    ):
        s = re.sub(r'[@\\]%s(?:\{\.\w+\})?\s?(.*?)\s?[@\\]%s' % (start_, end_),
                   r"```\n\1\n```\n", s, flags=re.DOTALL)

    s = re.sub(r'[@\\](?:end)?htmlonly\s+', r'', s)

    # These commands are always prefixed with an @ sign.
    s = re.sub(r'@[{}]\s*', r'', s)

    # Doxygen list commands.
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
    s = re.sub(r'[@\\]default\s+', r'\n$*Default:* ', s)

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
    for cmd_ in [
        'dir',
        'file',
    ]:
        s = re.sub(r'[@\\]%s( +[\w:./]+)?\s+' % cmd_, r'', s)

    # Remove these commands and their one optional single-line argument.
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
    s = re.sub(r'[@\\]headerfile\s+[\w:.]+( +[\w:.]+)?\s+', r'', s)

    # Remove these commands and their one single-word argument and one
    # optional single-line argument.
    for cmd_ in [
        'addtogroup',
        'weakgroup',
    ]:
        s = re.sub(r'[@\\]%s\s+[\w:.]( +.*)?\s+' % cmd_, r'', s)

    # Remove these commands and their one single-word argument and one
    # single-line argument. Ordering is significant for command names with a
    # common prefix.
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
    s = re.sub(r'(\s+)%(\S+)', r'\1\2', s)

    # HTML tags. Support both lowercase and uppercase tags.
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

    s = s.replace('``true``', '``True``')
    s = s.replace('``false``', '``False``')

    # Exceptions
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

    # Doxygen escaped characters.
    s = re.sub(r'[@\\]n\s+', r'\n\n', s)

    # Ordering of ---, --, @, and \ is significant.
    for escaped_ in (
        '---',
        '--',
        '::',
        '\.',
        '"',
        '&',
        '#',
        '%',
        '<',
        '>',
        '\$',
        '@',
        '\\\\',
    ):
        s = re.sub(r'[@\\](%s)' % escaped_, r'\1', s)

    # Reflow text where appropriate.
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
            for y in re.split(r'(?: *\n *){2,}', x):
                lines = re.split(r'(?: *\n *)', y)
                # Do not reflow lists or section headings.
                if (re.match('^(?:[*+\-]|[0-9]+[.)]) ', lines[0]) or
                    (len(lines) > 1 and
                     (lines[1] == '=' * len(lines[0]) or
                      lines[1] == '-' * len(lines[0])))):
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


def get_name_chain(cursor):
    """
    Extracts the pieces for a namespace-qualified name for a symbol.
    """
    name_chain = [utf8(cursor.spelling)]
    p = cursor.semantic_parent
    while p and p.kind != CursorKind.TRANSLATION_UNIT:
        piece = utf8(p.spelling)
        name_chain.insert(0, piece)
        p = p.semantic_parent
    # Do not try to specify names for anonymous structs.
    while '' in name_chain:
        name_chain.remove('')
    return tuple(name_chain)


class SymbolTree(object):
    """
    Contains symbols that (a) may have 0 or more pieces of documentation and
    (b) may have child objects.
    """

    def __init__(self):
        self.root = SymbolTree.Node()

    def get_node(self, name_chain):
        """
        Gets symbol node for a name chain, creating a fresh node if
        necessary.
        """
        node = self.root
        for piece in name_chain:
            node = node.get_child(piece)
        return node

    class Node(object):
        """Node for a given name chain."""
        def __init__(self):
            # First encountered occurence of a symbol when extracting, used to
            # label symbols that do not have documentation. Will only be None
            # for the root node.
            self.first_symbol = None
            # May be empty if no documented symbols are present.
            self.doc_symbols = []
            # Maps name to child nodes.
            self.children_map = defaultdict(SymbolTree.Node)

        def get_child(self, piece):
            return self.children_map[piece]


def extract(include_file_map, cursor, symbol_tree):
    """
    Extracts libclang cursors and add to a symbol tree.
    """
    if cursor.kind == CursorKind.TRANSLATION_UNIT:
        for i in cursor.get_children():
            extract(include_file_map, i, symbol_tree)
        return
    assert cursor.location.file is not None
    filename = utf8(cursor.location.file.name)
    include = include_file_map.get(filename)
    line = cursor.location.line
    if include is None:
        return
    name_chain = get_name_chain(cursor)
    if not is_accepted_cursor(cursor, name_chain):
        return
    node = None

    def get_node():
        node = symbol_tree.get_node(name_chain)
        if node.first_symbol is None:
            node.first_symbol = Symbol(
                cursor, name_chain, include, line, None)
        return node

    if cursor.kind in RECURSE_LIST:
        if node is None:
            node = get_node()
        for i in cursor.get_children():
            extract(include_file_map, i, symbol_tree)
    if cursor.kind in PRINT_LIST:
        if node is None:
            node = get_node()
        if len(cursor.spelling) > 0:
            comment = utf8(
                cursor.raw_comment) if cursor.raw_comment is not None else ''
            comment = process_comment(comment)
            symbol = Symbol(cursor, name_chain, include, line, comment)
            node.doc_symbols.append(symbol)


def choose_doc_var_names(symbols):
    """
    Given a list of Symbol objects for a single doc struct, chooses meaningful,
    unambiguous, terse names for them.  Returns a matching list of strings.  If
    a list element is None instead of str, then that symbol should be skipped
    (its documentation comment should *not* be emitted in this tool's output).
    """
    if len(symbols) == 0:
        return []

    # We will repeatedly frob this `result` list until it's good enough.
    result = []

    # If we can't find a good answer then only document the first symbol, using
    # a variable name that the user would be unable to accidentally refer to.
    # (This leaves evidence in generated documentation about what happened.)
    failure_result = [None] * len(symbols)
    failure_result[0] = "doc_was_unable_to_choose_unambiguous_names"

    def is_unique(candidate_result):
        # Are the non-None names in a candidate_result unique?
        trimmed = [x for x in candidate_result if x is not None]
        return len(trimmed) == len(set(trimmed))

    def specialize_well_known_doc_var_names():
        # Force well-known methods to have well-known names.
        nonlocal symbols, result
        for i, cursor in enumerate([s.cursor for s in symbols]):
            if "@exclude_from_pydrake_mkdoc" in symbols[i].comment:
                # Allow the user to opt-out this symbol from the documentation.
                # This is useful when forming unique constexpr names is
                # otherwise very difficult.  (Sometimes, C++ has *many* more
                # static-typing convenience overloads that pydrake really
                # needs, such as various kinds of Eigen<> template magic.)
                result[i] = None
                continue
            elif len(symbols[i].comment) == 0 and not (
                    cursor.is_default_constructor() and (
                        len(cursor.type.argument_types()) == 0)):
                # Ignore (almost all) overloads without docstrings.
                #
                # This is convenient for things like deprecated methods or
                # stubs (which are often no longer documented), where they
                # could otherwise pollute the overload naming set and cause us
                # to declare many "ambiguous name" failures.
                #
                # However, if a default constructor exists, we should always
                # provide a constexpr for it even if the user didn't write a
                # formatted API comment, so that our constant names stay
                # durable and so that function always participates in the
                # overload naming set.
                result[i] = None
                continue
            elif any([symbols[i].comment == x.comment for x in symbols[:i]]):
                # If a subsequent overload's API comment *exactly* matches a
                # prior overload's comment, the first overload's name wins.
                # This is important because when a function has separate
                # declaration and definition, we see its symbol *twice* in our
                # overload set, which would defeat our disambiguation
                # heuristics.  (Trying to cull the separate definition is not
                # tractable given clang's python bindings.)  This rule is
                # occasionally also useful for distinct function declarations
                # that nevertheless have identical documentation.
                result[i] = None
                continue
            elif cursor.is_copy_constructor():
                # Here, the semantics are distinct ("special member function")
                # so we should never use the "how many arguments" or "what are
                # the argument types" heuristics.
                result[i] = "doc_copy"
            elif cursor.is_move_constructor():
                # Here, the semantics are distinct ("special member function")
                # so we should never use the "how many arguments" or "what are
                # the argument types" heuristics.
                result[i] = "doc_move"
            elif (  # Look for a constructor like Foo<T>(const Foo<U>&).
                  cursor.kind == CursorKind.FUNCTION_TEMPLATE and
                  cursor.semantic_parent.kind == CursorKind.CLASS_TEMPLATE and
                  re.search(r"^(.*)<T>\(const \1<U> *&\)$",
                            utf8(cursor.displayname))):
                # Special case for scalar conversion constructors; we want to
                # have a nice short name for these, that doesn't necessarily
                # conflte with any *other* 1-argument constructor.
                result[i] = "doc_copyconvert"
            else:
                # If no special cases matched, leave the name alone.
                continue
            # A special case *did* match (we didn't hit the "else" above.)
            # When we have more than one identical well-known name (e.g,
            # separate declaration and definition doc_copy), use the first.
            assert result[i] is not None
            if result[i] in result[:i]:
                result[i] = None

    # Try the simplest naming choice -- call everything "doc".  If this makes
    # things unique (once the well-known heuristics are applied), ship it.
    result = ["doc"] * len(symbols)
    specialize_well_known_doc_var_names()
    if is_unique(result):
        if not any(result):
            # Always generate a constexpr when there are no overloads, even if
            # it's empty.  That way, pydrake can refer to the constant and any
            # future (single) API comment added to C++ will work automatically.
            result[0] = "doc"
        return result

    # All of the below heuristics only work for function overloads.
    if symbols[0].cursor.type.kind != TypeKind.FUNCTIONPROTO:
        return failure_result

    # Find the function argument types and (maybe) names.
    #
    # For FUNCTION_TEMPLATE symbols, get_arguments() is always empty (though
    # for FUNCTION_DECL it works).  So, we use argument_types() to get a
    # reliable count of arguments, use get_arguments() only for the names.
    #
    # These list-of-lists are indexed by [#overload][#argument].
    overload_arg_types = [
        [utf8(t.spelling) for t in s.cursor.type.argument_types()]
        for s in symbols
    ]
    overload_arg_names = [
        [utf8(a.spelling) for a in s.cursor.get_arguments()]
        for s in symbols
    ]

    # The argument count might be sufficient to disambiguate.
    result = ["doc_{}args".format(len(types)) for types in overload_arg_types]
    specialize_well_known_doc_var_names()
    if is_unique(result):
        return result

    # The parameter names (falling back to the parameter type, when we don't
    # know the name) might be sufficient to disambiguate.
    for i, arg_types in enumerate(overload_arg_types):
        if result[i] is None:
            continue
        arg_names = overload_arg_names[i] or [""] * len(arg_types)
        for arg_name, arg_type in zip(arg_names, arg_types):
            token = arg_name or sanitize_name(arg_type).replace("_", "")
            result[i] = result[i] + "_" + token
    specialize_well_known_doc_var_names()
    if is_unique(result):
        return result

    # Adding in the const-ness might be sufficient to disambiguate.
    for i, sym in enumerate(symbols):
        if result[i] is None:
            continue
        if sym.cursor.is_const_method():
            result[i] = result[i] + "_const"
        else:
            result[i] = result[i] + "_nonconst"
    specialize_well_known_doc_var_names()
    if is_unique(result):
        return result

    # As a last resort, return a random one, with a bogus name.
    return failure_result


def print_symbols(f, name, node, level=0):
    """
    Prints C++ code for releveant documentation.
    """
    indent = '  ' * level

    def iprint(s):
        f.write((indent + s).rstrip() + "\n")

    name_var = name
    if not node.first_symbol:
        assert level == 0
        full_name = name
    else:
        name_chain = node.first_symbol.name_chain
        assert name == name_chain[-1]
        full_name = "::".join(name_chain)
        # Override variable.
        if node.first_symbol.cursor.kind == CursorKind.CONSTRUCTOR:
            name_var = "ctor"

    name_var = sanitize_name(name_var)
    # We may get empty symbols if `libclang` produces warnings.
    assert len(name_var) > 0, node.first_symbol.sorting_key()
    iprint('// Symbol: {}'.format(full_name))
    modifier = ""
    if level == 0:
        modifier = "constexpr "
    iprint('{}struct /* {} */ {{'.format(modifier, name_var))
    # Print documentation items.
    symbol_iter = sorted(node.doc_symbols, key=Symbol.sorting_key)
    doc_vars = choose_doc_var_names(symbol_iter)
    for symbol, doc_var in zip(symbol_iter, doc_vars):
        if doc_var is None:
            continue
        assert name_chain == symbol.name_chain
        delim = "\n"
        if "\n" not in symbol.comment and len(symbol.comment) < 40:
            delim = " "
        iprint('  // Source: {}:{}'.format(symbol.include, symbol.line))
        iprint('  const char* {} ={}R"""({})""";'.format(
            doc_var, delim, symbol.comment))
    # Recurse into child elements.
    keys = sorted(node.children_map.keys())
    for key in keys:
        child = node.children_map[key]
        print_symbols(f, key, child, level=level + 1)
    iprint('}} {};'.format(name_var))


class FileDict(object):
    """
    Provides a dictionary that hashes based on a file's true path.
    """

    def __init__(self, items=[]):
        self._d = {self._key(file): value for file, value in items}

    def _key(self, file):
        return os.path.realpath(os.path.abspath(file))

    def get(self, file, default=None):
        return self._d.get(self._key(file), default)

    def __contains__(self, file):
        return self._key(file) in self._d

    def __getitem__(self, file):
        key = self._key(file)
        return self._d[key]

    def __setitem__(self, file, value):
        key = self._key(file)
        self._d[key] = value


def main():
    parameters = ['-x', 'c++', '-D__MKDOC_PY__']
    filenames = []

    if platform.system() == 'Darwin':
        dev_path = '/Applications/Xcode.app/Contents/Developer/'
        lib_dir = dev_path + 'Toolchains/XcodeDefault.xctoolchain/usr/lib/'
        sdk_dir = dev_path + 'Platforms/MacOSX.platform/Developer/SDKs'
        libclang = lib_dir + 'libclang.dylib'

        if os.path.exists(libclang):
            cindex.Config.set_library_path(os.path.dirname(libclang))

        if os.path.exists(sdk_dir):
            sysroot_dir = os.path.join(sdk_dir, next(os.walk(sdk_dir))[1][0])
            parameters.append('-isysroot')
            parameters.append(sysroot_dir)

    quiet = False
    std = '-std=c++11'
    root_name = 'mkdoc_doc'
    ignore_patterns = []
    output_filename = None

    for item in sys.argv[1:]:
        if item == '-quiet':
            quiet = True
        elif item.startswith('-output='):
            output_filename = item[len('-output='):]
        elif item.startswith('-std='):
            std = item
        elif item.startswith('-root-name='):
            root_name = item[len('-root-name='):]
        elif item.startswith('-exclude-hdr-patterns='):
            ignore_patterns.append(item[len('-exclude-hdr-patterns='):])
        elif item.startswith('-'):
            parameters.append(item)
        else:
            filenames.append(item)

    parameters.append(std)

    if output_filename is None or len(filenames) == 0:
        eprint('Syntax: %s -output=<file> [.. a list of header files ..]'
               % sys.argv[0])
        sys.exit(1)

    f = open(output_filename, 'w')
    # N.B. We substitute the `GENERATED FILE...` bits in this fashion because
    # otherwise Reviewable gets confused.
    f.write('''#pragma once

// {0} {1}
// This file contains docstrings for the Python bindings that were
// automatically extracted by mkdoc.py.

#if defined(__GNUG__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

'''.format('GENERATED FILE', 'DO NOT EDIT'))

    # Determine project include directories.
    # N.B. For simplicity when using with Bazel, we do not try to get canonical
    # file paths for determining include files.
    include_paths = []
    for param in parameters:
        # Only check for normal include directories.
        if param.startswith("-I"):
            include_paths.append(param[2:])
    # Use longest include directories first to get shortest include file
    # overall.
    include_paths = list(sorted(include_paths, key=len))[::-1]
    include_files = []
    # Create mapping from filename to include file.
    include_file_map = FileDict()
    for filename in filenames:
        for include_path in include_paths:
            prefix = include_path + "/"
            if filename.startswith(prefix):
                include_file = filename[len(prefix):]
                break
        else:
            raise RuntimeError(
                "Filename not incorporated into -I includes: {}".format(
                    filename))
        for p in ignore_patterns:
            if fnmatch(include_file, p):
                break
        else:
            include_files.append(include_file)
            include_file_map[filename] = include_file
    assert len(include_files) > 0
    # Generate the glue include file, which will include all relevant include
    # files, and parse. Add a unique prefix so we do not leak accidentally leak
    # in paths in `/tmp`.
    dir_prefix = mkdtemp(prefix="drake_mkdoc_")
    glue_include_file = NamedTemporaryFile(
        'w', prefix="glue_include_file_", dir=dir_prefix)
    with glue_include_file:
        for include_file in sorted(include_files):
            line = "#include \"{}\"".format(include_file)
            glue_include_file.write(line + "\n")
            f.write("// " + line + "\n")
        f.write("\n")
        glue_include_file.flush()
        if not quiet:
            eprint("Parse headers...")
        index = cindex.Index(
            cindex.conf.lib.clang_createIndex(False, True))
        translation_unit = index.parse(glue_include_file.name, parameters)
    os.rmdir(dir_prefix)
    # Extract symbols.
    if not quiet:
        eprint("Extract relevant symbols...")
    symbol_tree = SymbolTree()
    extract(include_file_map, translation_unit.cursor, symbol_tree)
    # Write header file.
    if not quiet:
        eprint("Writing header file...")
    try:
        print_symbols(f, root_name, symbol_tree.root)
    except UnicodeEncodeError as e:
        # User-friendly error for #9903.
        print("""
Encountered unicode error: {}
If you are on Ubuntu, please ensure you have en_US.UTF-8 locales generated:
    sudo apt-get install --no-install-recommends  locales
    sudo locale-gen en_US.UTF-8
""".format(e), file=sys.stderr)
        sys.exit(1)

    f.write('''
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
''')


if __name__ == '__main__':
    main()
