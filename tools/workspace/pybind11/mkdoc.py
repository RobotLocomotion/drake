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
#  Syntax:
#     mkdoc.py -output=<file> [-I<path> ..] [-quiet] [.. header files ..]
#
#  Extract documentation from C++ header files to use it in Python bindings
#

from collections import OrderedDict, defaultdict
from fnmatch import fnmatch
import os
import platform
import re
import shutil
import subprocess
import sys

from xml.dom import minidom
import xml.etree.ElementTree as ET
from clang import cindex
from clang.cindex import AccessSpecifier, CursorKind, TypeKind

from drake.tools.workspace.pybind11.mkdoc_comment import process_comment

from drake.tools.workspace.pybind11.libclang_setup import add_library_paths


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
    'DrakeDefaultCopyAndMoveAndAssign_DoAssign',
    'Eigen',
    'detail',
    'dev',
    'google',
    'internal',
    'std',
    'tinyxml2',
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


class Symbol:
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
    Determines if a symbol should be visited or not, given the cursor and the
    name chain.
    """
    name = utf8(cursor.spelling)
    # N.B. See TODO in `get_name_chain`.
    for piece in name_chain + (name,):
        if piece in SKIP_RECURSE_NAMES:
            return False
    for skip_partial_name in SKIP_PARTIAL_NAMES:
        if skip_partial_name in name:
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


def extract_comment(cursor, deprecations):
    # Returns the cursor's docstring INCLUDING any deprecation text.

    # Start with the cursor's docstring.
    result = ''
    if cursor.raw_comment is not None:
        result = utf8(cursor.raw_comment)

    # Look for a DRAKE_DEPRECATED macro.
    c = cursor  # The cursor whose deprecation macro we want to find.
    found = None  # The DRAKE_DEPRECATED cursor associated with `c`.
    possible_d = [
        d for d in deprecations
        if d.extent.start.file.name == c.extent.start.file.name
    ]

    # For a method declaration, the extent-begin-column for both will be
    # identical and the MACRO_INSTATIATION will end immediately prior to
    # the FUNCTION_DECL begin.
    for d in possible_d:
        if all([d.extent.start.column == c.extent.start.column,
                (d.extent.end.line + 1) == c.extent.start.line]):
            found = d
            break

    # For a class declaration, the MACRO_INSTATIATION extent will lie fully
    # within the CLASS_DECL extent, near the top.  Allow up to 5 lines between
    # the `class Foo` or `template <> class Foo` and the DRAKE_DEPRECATED macro
    # so that we're sure NOT to match a deprecated inline method near the top
    # of the class, but we DO allow various whitespace arrangements of template
    # parameters, class decl, and macro.
    for d in possible_d:
        if all([d.extent.start.line >= c.extent.start.line,
                d.extent.start.line <= (c.extent.start.line + 5),
                d.extent.end.line <= c.extent.end.line]):
            found = d
            break

    # If no deprecations matched, we are done.
    if not found:
        return result

    # Extract the DRAKE_DEPRECATED macro arguments.
    tokens = [x.spelling for x in found.get_tokens()]
    assert len(tokens) >= 6, tokens
    assert tokens[0] == b'DRAKE_DEPRECATED', tokens
    assert tokens[1] == b'(', tokens
    assert tokens[3] == b',', tokens
    assert tokens[-1] == b')', tokens
    removal_date = utf8(tokens[2])[1:-1]  # 1:-1 to strip quotes.
    message = "".join([
        utf8(x)[1:-1]
        for x in tokens[4:-1]
    ])

    # Append the deprecation text.
    result += (
        r" (Deprecated.) \deprecated {} "
        "This will be removed from Drake on or after {}.").format(
            message, removal_date)

    return result


def get_name_chain(cursor):
    """
    Extracts the pieces for a namespace-qualified name for a symbol.
    """
    # TODO(eric.cousineau): Try to restrict the name_chain to end with name. I
    # briefly tried this once by culling based on accepted cursors, but lost
    # needed symbols because of it.
    name = utf8(cursor.spelling)
    name_chain = [name]
    p = cursor.semantic_parent
    while p and p.kind != CursorKind.TRANSLATION_UNIT:
        piece = utf8(p.spelling)
        name_chain.insert(0, piece)
        p = p.semantic_parent
    # Do not try to specify names for anonymous structs.
    while '' in name_chain:
        name_chain.remove('')
    return tuple(name_chain)


class SymbolTree:
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

    class Node:
        """Node for a given name chain."""

        def __init__(self):
            # First encountered occurrence of a symbol when extracting, used to
            # label symbols that do not have documentation. Will only be None
            # for the root node.
            self.first_symbol = None
            # May be empty if no documented symbols are present.
            self.doc_symbols = []
            # Maps name to child nodes.
            self.children_map = defaultdict(SymbolTree.Node)

        def get_child(self, piece):
            return self.children_map[piece]


def extract(include_file_map, cursor, symbol_tree, deprecations=None):
    """
    Extracts libclang cursors and add to a symbol tree.
    """
    if cursor.kind == CursorKind.TRANSLATION_UNIT:
        deprecations = []
        for i in cursor.get_children():
            if i.kind == CursorKind.MACRO_DEFINITION:
                continue
            if i.kind == CursorKind.MACRO_INSTANTIATION:
                if i.spelling == b'DRAKE_DEPRECATED':
                    deprecations.append(i)
                continue
            extract(include_file_map, i, symbol_tree, deprecations)
        return
    assert cursor.location.file is not None, cursor.kind
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
            extract(include_file_map, i, symbol_tree, deprecations)
    if cursor.kind in PRINT_LIST:
        if node is None:
            node = get_node()
        if len(cursor.spelling) > 0:
            comment = extract_comment(cursor, deprecations)
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
            elif "@pydrake_mkdoc_identifier" in symbols[i].comment:
                comment = symbols[i].comment
                # Allow the user to manually specify a doc_foo identifier.
                match = re.search(
                    r"@pydrake_mkdoc_identifier\{(.*?)\}",
                    comment)
                if not match:
                    raise RuntimeError(
                        "Malformed pydrake_mkdoc_identifier in " + comment)
                (identifier,) = match.groups()
                result[i] = "doc_" + identifier
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
                cursor.kind == CursorKind.FUNCTION_TEMPLATE
                and cursor.semantic_parent.kind == CursorKind.CLASS_TEMPLATE
                and re.search(r"^(.*)<T>\(const \1<U> *&\)$",
                              utf8(cursor.displayname))):
                # Special case for scalar conversion constructors; we want to
                # have a nice short name for these, that doesn't necessarily
                # conflte with any *other* 1-argument constructor.
                result[i] = "doc_copyconvert"
            elif "\nDeprecated:" in symbols[i].comment:
                result[i] = "doc_deprecated" + result[i][3:]
                # Don't consolidate as if this were a "well known" name.
                continue
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


# TODO(m-chaturvedi): Refactor this to not use stack
def print_symbols(f, name, node, level=0, *, tree_parser_doc,
                  tree_parser_xpath, ignore_dirs_for_coverage):
    """
    Prints C++ code for relevant documentation.
    """
    indent = '  ' * level

    def iprint(s):
        f.write((indent + s).rstrip() + "\n")

    name_var = name
    if not node.first_symbol:
        assert level == 0, name_var
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

    root = tree_parser_xpath[-1]
    kind = node.first_symbol.cursor.kind if node.first_symbol else None

    tree_parser_doc.append(name_var)

    new_ele = None
    # Print documentation items.
    symbol_iter = sorted(node.doc_symbols, key=Symbol.sorting_key)
    doc_vars = choose_doc_var_names(symbol_iter)
    #  New element in the XML tree.
    new_ele = None

    for symbol, doc_var in zip(symbol_iter, doc_vars):
        if doc_var is None:
            continue
        assert name_chain == symbol.name_chain
        comment = re.sub(
            r'@pydrake_mkdoc[a-z_]*\{.*\}', '',
            symbol.comment)
        delim = "\n"
        if "\n" not in comment and len(comment) < 40:
            delim = " "
        iprint('  // Source: {}:{}'.format(symbol.include, symbol.line))
        iprint('  const char* {} ={}R"""({})""";'.format(
            doc_var, delim, comment.strip()))

        tree_doc_var = ".".join(tree_parser_doc + [doc_var])

        ignore_xpath = False
        if ignore_dirs_for_coverage:
            ignore_xpath = symbol.include.startswith(ignore_dirs_for_coverage)

        new_ele = ET.SubElement(root, "Node", {
            "kind": str(kind),
            "name": name_var,
            "full_name": full_name,
            "ignore": str(int(ignore_xpath)),
            "doc_var": tree_doc_var,
            "file_name": symbol.include,
            })
    # If the node has no doc_var's
    if new_ele is None:
        new_ele = ET.SubElement(root, "Node", {
            "kind": str(kind),
            "name": name_var,
            "full_name": full_name,
            "ignore": "",
            "doc_var": "",
            "file_name": "",
            })

    tree_parser_xpath.append(new_ele)
    # Recurse into child elements.
    keys = sorted(node.children_map.keys())
    for key in keys:
        child = node.children_map[key]
        tree_parser_args = {
                "tree_parser_doc": tree_parser_doc,
                "tree_parser_xpath": tree_parser_xpath,
                "ignore_dirs_for_coverage": ignore_dirs_for_coverage
            }
        print_symbols(f, key, child, level=level + 1, **tree_parser_args)
    iprint('}} {};'.format(name_var))

    tree_parser_doc.pop()
    tree_parser_xpath.pop()


class FileDict:
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


def prettify(elem):
    """Return a pretty-printed XML string for the Element.
    """
    rough_string = ET.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def main():
    parameters = ['-x', 'c++', '-D__MKDOC_PY__']
    add_library_paths(parameters)
    filenames = []

    quiet = False
    std = '-std=c++11'
    root_name = 'mkdoc_doc'
    ignore_patterns = []
    output_filename = None
    output_filename_xml = None

    # TODO(m-chaturvedi): Consider using argparse.
    for item in sys.argv[1:]:
        if item == '-quiet':
            quiet = True
        elif item.startswith('-output='):
            output_filename = item[len('-output='):]
        elif item.startswith('-output_xml='):
            output_filename_xml = item[len('-output_xml='):]
        elif item.startswith('-std='):
            std = item
        elif item.startswith('-ignore-dirs-for-coverage='):
            ignore_dir_str = item[len('-ignore-dirs-for-coverage='):]
            ignore_dirs_for_coverage = None
            if ignore_dir_str:
                ignore_dirs_for_coverage = tuple(ignore_dir_str.split(','))
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

    f = open(output_filename, 'w', encoding='utf-8')
    f_xml = None
    if output_filename_xml is not None:
        f_xml = open(output_filename_xml, 'w')

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
    used_ignore_patterns = set()
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
                used_ignore_patterns.add(p)
                break
        else:
            include_files.append(include_file)
            include_file_map[filename] = include_file
    assert len(include_files) > 0
    unused_ignore_patterns = set(ignore_patterns) - used_ignore_patterns
    if unused_ignore_patterns:
        print(f"Unused ignore patterns: {unused_ignore_patterns}")
    # Generate the glue include file, which will include all relevant include
    # files, and parse. Use a tempdir that is relative to the output file for
    # usage with Bazel.
    tmpdir = output_filename + ".tmp_artifacts"
    os.mkdir(tmpdir)
    glue_filename = os.path.join(tmpdir, "mkdoc_glue.h")
    with open(glue_filename, 'w') as glue_f:
        # As the first line of the glue file, include a C++17 standard library
        # file to sanity check that it's working, before we start processing
        # the user headers.
        glue_f.write("#include <optional>\n")
        # Add the includes to the glue, and as comments in the output.
        for include_file in sorted(include_files):
            line = "#include \"{}\"".format(include_file)
            glue_f.write(line + "\n")
            f.write("// " + line + "\n")
        f.write("\n")
        glue_f.flush()
        if not quiet:
            eprint("Parse headers...")
        index = cindex.Index(
            cindex.conf.lib.clang_createIndex(False, True))
        translation_unit = index.parse(
            glue_filename, parameters,
            options=cindex.TranslationUnit.PARSE_DETAILED_PROCESSING_RECORD)
        if not translation_unit:
            raise RuntimeError(
                "Parsing headers using the clang library failed")
        # If there is an error on line 1, that means the C++ standard library
        # include paths are broken.
        if translation_unit.diagnostics:
            if translation_unit.diagnostics[0].location.line == 1:
                try:
                    # Use '###' to dump Clang's include paths to stdout.
                    index.parse("foo", parameters + ["-###"])
                except Exception:
                    pass
                raise RuntimeError(
                    ("The operating system's C++ standard library is not "
                     "installed correctly."))
        severities = [
            diagnostic.severity for diagnostic in translation_unit.diagnostics
            if diagnostic.severity >= cindex.Diagnostic.Error
        ]
        if severities:
            raise RuntimeError(
                ("Parsing headers using the clang library failed with {} "
                 "error(s) and {} fatal error(s)").format(
                     severities.count(cindex.Diagnostic.Error),
                     severities.count(cindex.Diagnostic.Fatal)))
    shutil.rmtree(tmpdir)
    # Extract symbols.
    if not quiet:
        eprint("Extract relevant symbols...")
    symbol_tree = SymbolTree()
    extract(include_file_map, translation_unit.cursor, symbol_tree)
    # Write header file.
    if not quiet:
        eprint("Writing header file...")
    try:
        tree_parser = {"tree_parser_doc": [],
                       "tree_parser_xpath": [ET.Element("Root")],
                       "ignore_dirs_for_coverage": ignore_dirs_for_coverage}

        print_symbols(f, root_name, symbol_tree.root, **tree_parser)
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
    if f_xml is not None:
        f_xml.write(prettify(tree_parser["tree_parser_xpath"][0]))


if __name__ == '__main__':
    main()
