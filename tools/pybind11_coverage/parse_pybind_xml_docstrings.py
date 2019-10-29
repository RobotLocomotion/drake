# Tested with Python 3
from lxml import etree as ET
import pandas
import os

assert(__import__('sys').version_info[0] == 3)
XPATHS = {
    "class_decl": ".//Node[@kind='CursorKind.CLASS_DECL' or "
    "@kind='CursorKind.CLASS_TEMPLATE' or @kind='CursorKind.STRUCT_DECL']"
    "[@ignore='0']",
    "file_names": ".//Node",
    "doc_var_xpath": ".//Node[@ignore='0' and @doc_var]",
}

all_kinds = [
    'CursorKind.CLASS_DECL',
    'CursorKind.CLASS_TEMPLATE',
    'CursorKind.CONSTRUCTOR',
    'CursorKind.CONVERSION_FUNCTION',
    'CursorKind.CXX_METHOD',
    'CursorKind.ENUM_CONSTANT_DECL',
    'CursorKind.ENUM_DECL',
    'CursorKind.FIELD_DECL',
    'CursorKind.FUNCTION_DECL',
    'CursorKind.FUNCTION_TEMPLATE',
    'CursorKind.STRUCT_DECL',
    'CursorKind.TYPEDEF_DECL',
    'CursorKind.TYPE_ALIAS_DECL',
]

# Not used: {'CursorKind.FIELD_DECL', 'CursorKind.TYPEDEF_DECL'}
# Note that the figures given in the coverage column may not be the sum of the
# figures in the types column because we're not using two types.
print_kinds = {
    "Class": ['CursorKind.CLASS_DECL', 'CursorKind.STRUCT_DECL'],
    "Class Template": ["CursorKind.CLASS_TEMPLATE"],
    "Type Aliases": ['CursorKind.TYPE_ALIAS_DECL'],
    "Functions": [
        'CursorKind.FUNCTION_DECL', 'CursorKind.CONVERSION_FUNCTION'
        ],
    "Function Template": ['CursorKind.FUNCTION_TEMPLATE'],
    "Enums": ['CursorKind.ENUM_DECL', 'CursorKind.ENUM_CONSTANT_DECL'],
    "Methods": ['CursorKind.CONSTRUCTOR', 'CursorKind.CXX_METHOD'],
}


class Coverage(object):
    def __init__(self, num=0, den=0): self.num, self.den = num, den

    def __str__(self):
        if self.num == 0 and self.den == 0:
            return ""
        else:
            return "{}/{}".format(self.num, self.den)

    def __add__(self, other):
        return Coverage(self.num + other.num, self.den + other.den)

    def __eq__(self, ot): return self.num == ot.num and self.den == ot.den

    def val(self): return (float)(self.num) / self.den

    def __getattr__(self, attr):
        raise AttributeError("'{}' object has no attribute '{}'".format(
            self.__class__.__name__, attr))


class FileName(object):

    def __init__(self, s): self.s = str(s)

    def __str__(self): return self.s

    def __lt__(self, other):
        L1 = self.s.split(os.path.sep)
        L2 = other.s.split(os.path.sep)
        if len(L1) <= 1 or len(L2) <= 1:
            return self.s < other.s

        return L1[:-1] < L2[:-1] if L1[:-1] != L2[:-1] else L1[-1] < L2[-1]

    def __getattr__(self, attr):
        raise AttributeError("'{}' object has no attribute '{}'".format(
            self.__class__.__name__, attr))


def get_pandas_row_for_nodes(nodes, pybind_strings):
    """Gets a row of pandas table.  Extracts the kind and doc_var.

    :param nodes: The nodes for which the kind and doc_var is obtained.
    :param pybind_strings: The array containing the strings gotten from
    pybind parser.
    """
    found = [0] * len(all_kinds)
    total = [0] * len(all_kinds)
    ret = {}

    for node in nodes:
        kind = node.attrib["kind"]
        doc_var = node.attrib["doc_var"]
        ind = all_kinds.index(kind)
        total[ind] = total[ind] + 1
        found[ind] = found[ind] + (1 if doc_var in pybind_strings else 0)

    for kind, f, t in zip(all_kinds, found, total):
        ret[kind] = Coverage(f, t)

    ret["Coverage"] = Coverage(sum(found), sum(total))

    return ret


def get_class_coverage(root_node, pybind_strings, df):
    """Gets the coverage for root_node

    :root_node: Root node of a class
    :pybind_strings: Pybind docstring gotten from libclang parser
    :df: Pandas dataframe, initiated
    :returns: Pandas dataframe

    """
    for c in root_node:
        # All nodes which are not ignored and have a doc_var
        doc_var_nodes = c.xpath(XPATHS["doc_var_xpath"])
        row = get_pandas_row_for_nodes(doc_var_nodes, pybind_strings)

        row["ClassName"] = c.attrib["full_name"]
        df = df.append(row, ignore_index=True)
    return df


def get_file_coverage(file_nodes, pybind_strings, df):
    """Gets the file-wise coverage.  We first list the nodes inside a header
    file in a dictionary and then proceed.

    :param file_nodes: All nodes corresponding to a file
    :pybind_strings: Pybind docstring gotten from libclang parser
    :param df: Pandas dataframe
    """
    file_name_dict = {}
    for node in file_nodes:
        file_name = node.attrib["file_name"]
        if file_name in file_name_dict:
            file_name_dict[file_name].append(node)
        else:
            file_name_dict[file_name] = [node]

    for fn in file_name_dict:
        # All nodes which are not ignored and have a doc_var
        doc_nodes = [n for n in file_name_dict[fn] if "doc_var" in n.attrib]
        assert(len(doc_nodes) == len(file_name_dict[fn]))
        row = get_pandas_row_for_nodes(doc_nodes, pybind_strings)

        row["FileName"] = FileName(fn)
        df = df.append(row, ignore_index=True)

    return df


def setup_pandas(cols):
    """Sets up pandas a dataframe using the kinds of elements cindex finds.

    :returns: A pandas dataframe

    """
    df = pandas.DataFrame(columns=cols + [x for x in all_kinds])

    return df


def get_all_class_coverage(root, pybind_strings, csv_file, prune=True):
    class_nodes = root.xpath(XPATHS["class_decl"])
    df = setup_pandas(["ClassName", "Coverage"])
    df = get_class_coverage(class_nodes, pybind_strings, df)
    df_pruned = prune_dataframe(df, ["ClassName", "Coverage"])
    (df_pruned if prune else df).to_csv(csv_file, index=False)


def make_tree(file_coverage, sep="/"):
    """Makes an XML tree from list of paths.

    :param file_coverage: A dictionary of elements of type:
        <string> : <Coverage>
        where, filename is the string and Coverage is its coverage.
    :param sep: The path separator.
    """
    root = ET.Element("Root")
    SE = ET.SubElement
    for s in file_coverage:
        comps = str(s).split(os.sep)
        r = root
        for ind, c in enumerate(comps):
            if r.find(c) is None:
                num = file_coverage[s].num if c.endswith(".h") else 0
                den = file_coverage[s].den if c.endswith(".h") else 0
                r = SE(r, c, {
                               "num": str(num),
                               "den": str(den),
                               "name": (os.sep).join(comps[:ind+1])
                            })
            else:
                r = r.find(c)
    return root


def add_directory_coverage(df, csv_file="file_coverage.csv"):
    """Makes an XML tree from filenames and their coverage and then uses it
       to get directory coverage whenever there's a change in the immediate
       parent of the leaf (i.e. the header file) a row of the dataframe.

    :param df: Pandas dataframe containing the coverage details.
    """
    filenames, coverage = df.loc[:, "FileName"], df.loc[:, "Coverage"]
    file_coverage = dict(zip(filenames, coverage))
    root = make_tree(file_coverage)
    XP = root.xpath
    dirname = None
    final_row = {}

    for i, row in df.iterrows():
        total_num, total_den = 0, 0
        if dirname != os.path.dirname(str(row["FileName"])):
            dirname = os.path.dirname(str(row["FileName"]))
            total_num = sum([
                    int(n) for n in XP('.//{}/*/@num'.format(dirname))
                ])
            total_den = sum([
                    int(d) for d in XP('.//{}/*/@den'.format(dirname))
                ])
        row["DirCoverage"] = Coverage(total_num, total_den)

    cols = df.columns.tolist()
    [cols.remove(x) for x in ["FileName", "DirCoverage"]]

    for col in cols:
        final_row[col] = df[col].sum()

    sum([v for k, v in final_row.items() if k not in ["Coverage"]],
        Coverage(0, 0)) == final_row["Coverage"]

    final_row["FileName"] = "TOTAL"
    df = df.append(final_row, ignore_index=True)
    print("Coverage = {}".format(str(final_row["Coverage"])))
    df.to_csv(csv_file, index=False)
    return df


def prune_dataframe(df, keep_cols):
    """Since we are not printing all the variable types which are read, we will
       read from the dictionary `print_kinds` and return the dataframe.

    :param df: DataFrame to prune using print_kinds
    """
    new_df = pandas.DataFrame()
    new_df = df[keep_cols].copy()
    for key in print_kinds:
        val = print_kinds[key]
        new_df[key] = df.loc[:, val].sum(axis='columns')
    return new_df


def get_all_file_coverage(root, pybind_strings, csv_file, prune=True):
    file_nodes = root.xpath(XPATHS["doc_var_xpath"])
    df = setup_pandas(["DirCoverage", "FileName", "Coverage"])
    df = get_file_coverage(file_nodes, pybind_strings, df)
    df_sorted = df.sort_values(by=['FileName'])
    df_pruned = prune_dataframe(df_sorted, [
        "DirCoverage", "FileName", "Coverage"])
    final_df = add_directory_coverage(
            df_pruned if prune else df_sorted, csv_file)
    return final_df


def get_file_and_class_coverage(xml_file, file_coverage_csv,
                                class_coverage_csv, pybind_strings):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    get_all_class_coverage(root, pybind_strings,
                           class_coverage_csv, prune=True)
    file_coverage_df = get_all_file_coverage(
            root, pybind_strings, file_coverage_csv, prune=True)

    return file_coverage_df
