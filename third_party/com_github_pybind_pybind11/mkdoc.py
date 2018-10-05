#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
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
import sys
import re
from tempfile import NamedTemporaryFile
import textwrap

from clang import cindex
from clang.cindex import AccessSpecifier, CursorKind

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
    CursorKind.FIELD_DECL
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

SKIP_FULL_NAMES = [
    'Eigen',
    'detail',
    'google',
    'internal',
    'std',
    'tinyxml2',
]

SKIP_PARTIAL_NAMES = [
    'operator new',
    'operator delete',
    'operator=',
    'operator->',
    'operator<<',
    'operator>>',
]

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


def is_accepted_cursor(cursor):
    """
    Determines if a symbol should be visited or not.
    """
    name = utf8(cursor.spelling)
    if name in SKIP_FULL_NAMES:
        return False
    for bad in SKIP_PARTIAL_NAMES:
        if bad in name:
            return False
    if cursor.access_specifier in SKIP_ACCESS:
        return False
    # TODO(eric.cousineau): Remove `cursor.is_default_method()`? May make
    # things unstable.
    # TODO(eric.cousineau): Figure out how to strip forward declarations.
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
        if s.startswith('/*'):
            s = s[2:].lstrip('*')
        elif s.endswith('*/'):
            s = s[:-2].rstrip('*')
        elif s.startswith('///'):
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

    # Doxygen tags
    cpp_group = r'([\w:]+)'
    param_group = r'([\[\w:\]]+)'

    s = result
    s = re.sub(r'[@\\]c\s+%s' % cpp_group, r'``\1``', s)
    s = re.sub(r'[@\\]p\s+%s' % cpp_group, r'``\1``', s)
    s = re.sub(r'[@\\]a\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]e\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]em\s+%s' % cpp_group, r'*\1*', s)
    s = re.sub(r'[@\\]b\s+%s' % cpp_group, r'**\1**', s)
    s = re.sub(r'[@\\]ingroup\s+%s' % cpp_group, r'', s)
    s = re.sub(r'[@\\]param%s?\s+%s' % (param_group, cpp_group),
               r'\n\n$Parameter ``\2``:\n\n', s)
    s = re.sub(r'[@\\]tparam%s?\s+%s' % (param_group, cpp_group),
               r'\n\n$Template parameter ``\2``:\n\n', s)
    s = re.sub(r'[@\\]retval\s+%s' % cpp_group,
               r'\n\n$Returns ``\1``:\n\n', s)

    for in_, out_ in {
        'result': 'Returns',
        'returns': 'Returns',
        'return': 'Returns',
        'authors': 'Authors',
        'author': 'Authors',
        'copyright': 'Copyright',
        'date': 'Date',
        'note': 'Note',
        'remarks': 'Remark',
        'remark': 'Remark',
        'sa': 'See also',
        'see': 'See also',
        'extends': 'Extends',
        'throws': 'Throws',
        'throw': 'Throws'
    }.items():
        s = re.sub(r'[@\\]%s\s*' % in_, r'\n\n$%s:\n\n' % out_, s)

    s = re.sub(r'[@\\]details\s*', r'\n\n', s)
    s = re.sub(r'[@\\]brief\s*', r'', s)
    s = re.sub(r'[@\\]short\s*', r'', s)
    s = re.sub(r'[@\\]ref\s*', r'', s)

    s = re.sub(r'[@\\]code\s?(.*?)\s?[@\\]endcode',
               r"```\n\1\n```\n", s, flags=re.DOTALL)

    s = re.sub(r'%(\S+)', r'\1', s)

    # HTML/TeX tags
    s = re.sub(r'<tt>(.*?)</tt>', r'``\1``', s, flags=re.DOTALL)
    s = re.sub(r'<pre>(.*?)</pre>', r"```\n\1\n```\n", s, flags=re.DOTALL)
    s = re.sub(r'<em>(.*?)</em>', r'*\1*', s, flags=re.DOTALL)
    s = re.sub(r'<b>(.*?)</b>', r'**\1**', s, flags=re.DOTALL)
    s = re.sub(r'[@\\]f\$(.*?)[@\\]f\$', r'$\1$', s, flags=re.DOTALL)
    s = re.sub(r'<li>', r'\n\n* ', s)
    s = re.sub(r'</?ul>', r'', s)
    s = re.sub(r'</li>', r'\n\n', s)

    s = s.replace('``true``', '``True``')
    s = s.replace('``false``', '``False``')

    # Exceptions
    s = s.replace('std::bad_alloc', 'MemoryError')
    s = s.replace('std::domain_error', 'ValueError')
    s = s.replace('std::exception', 'RuntimeError')
    s = s.replace('std::invalid_argument', 'ValueError')
    s = s.replace('std::length_error', 'ValueError')
    s = s.replace('std::out_of_range', 'ValueError')
    s = s.replace('std::range_error', 'ValueError')

    # Re-flow text
    wrapper = textwrap.TextWrapper()
    wrapper.expand_tabs = True
    wrapper.replace_whitespace = True
    wrapper.drop_whitespace = True
    wrapper.width = 70
    wrapper.initial_indent = wrapper.subsequent_indent = ''

    result = ''
    in_code_segment = False
    for x in re.split(r'(```)', s):
        if x == '```':
            if not in_code_segment:
                result += '```\n'
            else:
                result += '\n```\n\n'
            in_code_segment = not in_code_segment
        elif in_code_segment:
            result += x.strip()
        else:
            for y in re.split(r'(?: *\n *){2,}', x):
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
    while p.kind != CursorKind.TRANSLATION_UNIT:
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
    if not is_accepted_cursor(cursor):
        return
    name_chain = None

    def get_node():
        name_chain = get_name_chain(cursor)
        node = symbol_tree.get_node(name_chain)
        if node.first_symbol is None:
            node.first_symbol = Symbol(
                cursor, name_chain, include, line, None)
        return name_chain, node

    if cursor.kind in RECURSE_LIST:
        if name_chain is None:
            name_chain, node = get_node()
        for i in cursor.get_children():
            extract(include_file_map, i, symbol_tree)
    if cursor.kind in PRINT_LIST:
        if name_chain is None:
            name_chain, node = get_node()
        if len(cursor.spelling) > 0:
            comment = utf8(
                cursor.raw_comment) if cursor.raw_comment is not None else ''
            comment = process_comment(comment)
            symbol = Symbol(cursor, name_chain, include, line, comment)
            node.doc_symbols.append(symbol)


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
    for i, symbol in enumerate(symbol_iter):
        assert name_chain == symbol.name_chain
        doc_var = "doc"
        if i > 0:
            doc_var += "_{}".format(i + 1)
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
        exit(-1)

    f = open(output_filename, 'w')
    # N.B. We substitue the `GENERATED FILE...` bits in this fashion because
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
    # files, and parse.
    with NamedTemporaryFile('w') as glue_include_file:
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
    # Extract symbols.
    if not quiet:
        eprint("Extract relevant symbols...")
    symbol_tree = SymbolTree()
    extract(include_file_map, translation_unit.cursor, symbol_tree)
    # Write header file.
    if not quiet:
        eprint("Writing header file...")
    print_symbols(f, root_name, symbol_tree.root)

    f.write('''
#if defined(__GNUG__)
#pragma GCC diagnostic pop
#endif
''')


if __name__ == '__main__':
    main()
