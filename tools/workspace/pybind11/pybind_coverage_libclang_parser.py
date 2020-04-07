# This file parses the python bindings files and extracts the doc variables
# using libclang.

# KNOWN ASSUMPTIONS:
# 1. No shadowing for line-wise variable replacement.
# 2. The `doc` variables are always defined like so:
#    `constexpr auto& doc = ...`
# 3. Anything that doesn't observe the line-wise replacement technique. For
#    example: There's a function to which the documentation variable is passed,
#    and the function is defined _before_ the call.  There can be other cases
#    well.
# 4. The replaced variables are from the list `REPLACE_VARIABLES`.
# 5. The doc variables used like so (ignore the whitespaces):
#    `, <doc_variable> )`

import logging
import re

from clang import cindex

from .libclang_setup import add_library_paths


REPLACE_VARIABLES = ["doc", "cls_doc", "var_doc", "enum_doc"]


class DocVariable:
    _val = None

    @classmethod
    def val(cls): return cls._val if cls._val else "pydrake_doc"


def var_value_search(var_name):
    """Defines the regex needed for extracting the values with which `var_name`
       is initialized.

    Args:
        var_name: The name of the variable which follows the convention
            (whitespace is ignored)
            `constexpr auto& <var_name> = ...`

    Returns:
        The regex hash needed for search.
    """
    re_for_search = {
        "start_tokens": ["constexpr", "auto", "&", var_name, "="],
        "value_regex": r'[\w.]+',
        "end_token": r';'
    }
    return re_for_search


def pydrake_doc_search():
    """Get the regex needed for finding locations where doc variables are used
       in pybind functions.  Locations such as:
       (whitespace is ignored)
       `..., pydrake_doc.<var_name> )`


    Args:

    Returns:
        The regex needed for subitution.
    """
    re_for_search = {
        "start_tokens": [","],
        "value_regex": (DocVariable.val() + r'[\w.]+'),
        "end_token": ")"
    }
    return re_for_search


def var_substitution_search(var_name):
    """Get the regex needed for finding locations to do substitutions for
       the value of the variables found above.

    Args:
        var_name: The variable name

    Returns:
    """
    re_for_search = {
        "start_token": r'[=,]',
        "value_regex": var_name,
        "end_token": r'[\.]'
    }
    return re_for_search


def get_var_value(re_search, tokens, i):
    """Gets the list of tokens between re_search["start_tokens"] and
    re_search["end_tokens"].

    Args:
        re_search: Dictionary with hash values `start_tokens` and `end_token`.
        tokens: list of tokens in which to look for `re_search`.
        i: starting index in the list of tokens.

    Returns: list of tokens found between `re_search["start_tokens"]` and
    `re_search["end_token"]`.
    """
    li = []
    i = i + len(re_search["start_tokens"])
    while(i < len(tokens) and tokens[i] != re_search["end_token"]):
        li.append(tokens[i])
        i = i + 1
    return li


def check_substitution_regex(tokens, i, value_li, var_name):
    re_for_search = var_substitution_search(var_name)
    st = re_for_search["start_token"]
    en = re_for_search["end_token"]

    tokens_match = False

    if re.match(st, tokens[i]) and len(tokens) > i + 2 and \
            re.match(en, tokens[i+2]):
        tokens_match = True

    if tokens_match and tokens[i+1] == var_name:
        tokens[i+1:i+2] = value_li


def replace_tokens(tokens, var_name):
    re_search = var_value_search(var_name)
    st_tokens = re_search["start_tokens"]
    value_li = None

    for i, t in enumerate(tokens):
        num_tokens = len(st_tokens)
        if tokens[i:(i + num_tokens)] == st_tokens:
            value_li = get_var_value(re_search, tokens, i)
            st = "".join(value_li)
            assert(re.match(re_search["value_regex"], st))

        elif value_li:
            check_substitution_regex(tokens, i, value_li, var_name)


def get_pydoc_strings(tokens):
    re_search = pydrake_doc_search()
    st_tokens = re_search["start_tokens"]
    rex = re_search["value_regex"]
    value_li = None
    pydoc_strings = []
    for i, t in enumerate(tokens):
        num_tokens = len(st_tokens)
        if tokens[i:(i + num_tokens)] == st_tokens:
            value_li = get_var_value(re_search, tokens, i)
            joined_li = "".join(value_li)
            if re.match(rex, joined_li):
                pydoc_strings.append(joined_li)
    return pydoc_strings


def write_to_file(arr, file_name):
    with open(file_name, "w") as f:
        [f.write(s + "\n") for s in arr]


def get_tokens(filename):
    tu = cindex.TranslationUnit.from_source(filename, ["-std=c++17"])
    FILE = tu.get_file(bytes(filename, 'utf8'))

    with open(filename, "r") as f:
        readlines = f.readlines()

    lines, cols = len(readlines), len(readlines[-1])
    st = cindex.SourceLocation.from_position(tu, FILE, 1, 1)
    en = cindex.SourceLocation.from_position(tu, FILE, lines, cols)
    extent = cindex.SourceRange.from_locations(st, en)
    tokens = tu.get_tokens(extent=extent)
    token_spellings = [t.spelling.decode('utf-8') for t in tokens if t.kind is
                       not cindex.TokenKind.COMMENT]

    return token_spellings


def replace_tokens_in_file(filename):
    token_spellings_original = get_tokens(filename)
    token_spellings = token_spellings_original[:]

    for x in REPLACE_VARIABLES:
        replace_tokens(token_spellings, x)
    pydoc_strings = get_pydoc_strings(token_spellings)

    write_to_file(token_spellings_original, "A.txt")
    write_to_file(token_spellings, "B.txt")
    return token_spellings, pydoc_strings


def get_docstrings_from_bindings(filenames, pydrake_doc_variable):
    """Given a list of pybind filenames, get the docstrings used in them

    Args:
        filenames: Names of the files

    Returns:
        List of the docstrings used
    """
    # TODO(eric.cousineau): Hoist side effects to main file?
    add_library_paths()
    logging.basicConfig(level=logging.INFO)
    DocVariable._val = pydrake_doc_variable

    array_for_all_files = []
    for f in filenames:
        logging.debug("On file: {}".format(f))
        _, final_array = replace_tokens_in_file(f)
        array_for_all_files = array_for_all_files + final_array
    return [x for x in array_for_all_files if x]
