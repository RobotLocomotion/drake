from lxml import etree as ET
import pandas
import os
import logging


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


class Coverage:
    def __init__(self, num=0, den=0):
        self.num = num
        self.den = den

    def __str__(self):
        if self.num == 0 and self.den == 0:
            return ""
        else:
            return "{}/{}".format(self.num, self.den)

    def __add__(self, other):
        return Coverage(self.num + other.num, self.den + other.den)

    def __eq__(self, ot): return self.num == ot.num and self.den == ot.den

    def __getattr__(self, attr):
        raise AttributeError("'{}' object has no attribute '{}'".format(
            self.__class__.__name__, attr))


class FileName:

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


class CommonUtils:
    def get_node_coverage(root_node, pybind_strings):
        """Go through the children of `root_node`, classify them and find their
        coverage.

        Args:
            root_node (xml node): The root node children of which are needed to
                be found coverage for.
            pybind_strings (list): List containing the strings gotten from
                the bindings' files.

        Returns:
            dict: A row for pandas dataframe.
        """
        found = [0] * len(all_kinds)
        total = [0] * len(all_kinds)
        ret = {}

        for node in root_node:
            kind = node.attrib["kind"]
            doc_var = node.attrib["doc_var"]
            ind = all_kinds.index(kind)
            total[ind] = total[ind] + 1
            found[ind] = found[ind] + (1 if doc_var in pybind_strings else 0)

        for kind, f, t in zip(all_kinds, found, total):
            ret[kind] = Coverage(f, t)

        ret["Coverage"] = Coverage(sum(found), sum(total))

        return ret

    def prune_dataframe(df, keep_cols):
        """Get new dataset with columns that we want to print to CSV.

        Since we are not printing all the variable types which are read,
        we will read from the dictionary `print_kinds` and return the
        dataframe, after summing up relevant columns.

        Args:
            df (pandas dataframe): DataFrame to prune using print_kinds
            keep_cols (list): The columns that we want to keep in the new
                dataframe.

        Returns:
            (pandas dataframe): The pruned dataframe
        """
        new_df = pandas.DataFrame()
        new_df = df[keep_cols].copy()
        for key in print_kinds:
            val = print_kinds[key]
            new_df[key] = df.loc[:, val].sum(axis='columns')
        return new_df


class ClassCoverage:

    def __init__(self, xml_file, pybind_strings, class_coverage_csv):
        """Constructor for `ClassCoverage` instance.

        Args:
                xml_file (str): XML file to process.
                pybind_strings (list): List containing the strings gotten from
                    the bindings' files.
                class_coverage_csv (str): Name of the CSV file to write class
                    coverage statistics in.

        """
        self.xml_root = ET.parse(xml_file).getroot()
        self.pybind_strings = pybind_strings
        self.csv = class_coverage_csv
        self.df = None
        self.df_pruned = None

    def write_coverage(self):
        """Writes class-wise coverage to a CSV file.

        """
        class_nodes = self.xml_root.xpath(XPATHS["class_decl"])
        self.df = pandas.DataFrame(
                columns=["ClassName", "Coverage"] + all_kinds)
        self.get_class_coverage(class_nodes)

        self.df_pruned.to_csv(self.csv, index=False)

    def get_class_coverage(self, class_nodes):
        """Goes through the relevant children of each element of `class_nodes`
        and then appends one row for each class to `df` with details of
        coverage of all kind of elements we're looking for

        Args:
                class_nodes (type): Nodes of class type

        """

        for c in class_nodes:
            # All nodes which are not ignored and have a doc_var
            doc_var_nodes = c.xpath(XPATHS["doc_var_xpath"])
            row = CommonUtils.get_node_coverage(
                    doc_var_nodes, self.pybind_strings)

            row["ClassName"] = c.attrib["full_name"]
            self.df = self.df.append(row, ignore_index=True)

        self.df_pruned = CommonUtils.prune_dataframe(
                self.df, ["ClassName", "Coverage"])


class FileCoverage:

    def __init__(self, xml_file, pybind_strings, file_coverage_csv):
        """Constructor for `FileCoverage` instance.

        Args:
                xml_file: XML file to process.
                pybind_strings (list): Documentation strings parsed from pybind
                    bindings.
                file_coverage_csv (str): Name of the CSV file to write file
                    coverage statistics in.

        """
        self.xml_root = ET.parse(xml_file).getroot()
        self.pybind_strings = pybind_strings
        self.csv = file_coverage_csv
        self.df = None
        self.df_pruned = None

    def write_coverage(self):
        """Writes file-wise coverage to a CSV file.

        """
        file_nodes = self.xml_root.xpath(XPATHS["doc_var_xpath"])
        self.df = pandas.DataFrame(
                columns=["DirCoverage", "FileName", "Coverage"] + all_kinds)

        self.get_file_coverage(file_nodes)

        df_sorted = self.df.sort_values(by=['FileName'])
        #  print(self.df.columns)
        self.df_pruned = CommonUtils.prune_dataframe(df_sorted, [
            "DirCoverage", "FileName", "Coverage"])

        self.add_directory_coverage()
        self.df_pruned.to_csv(self.csv, index=False)

    def get_file_coverage(self, file_nodes):
        """Gets the file-wise coverage.  We first list the nodes inside a
        header file in a dictionary and then proceed.

        Args:
                file_nodes: All nodes corresponding to a file.

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
            doc_nodes = [n for n in file_name_dict[fn]
                         if "doc_var" in n.attrib]

            assert(len(doc_nodes) == len(file_name_dict[fn]))
            row = CommonUtils.get_node_coverage(doc_nodes, self.pybind_strings)

            row["FileName"] = FileName(fn)
            self.df = self.df.append(row, ignore_index=True)

    def make_tree(self, file_coverage, sep="/"):
        """Makes an XML tree from list of paths.

        Args:
                file_coverage (dict): A dictionary of elements of type:
                    <string> : <Coverage>
                    where, filename is the string and Coverage is its coverage.

                sep (str): The path separator.

        Returns:
            ET.Element: The root node of the tree created from the paths.
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

    def add_directory_coverage(self):
        """Makes an XML tree from filenames and their coverage and then uses it
           to get directory coverage whenever there's a change in the immediate
           parent of the leaf (i.e. the header file) a row is added to the
           dataframe.
        """
        logging.basicConfig(level=logging.INFO)
        df = self.df_pruned
        filenames, coverage = df.loc[:, "FileName"], df.loc[:, "Coverage"]
        file_coverage = dict(zip(filenames, coverage))
        root = self.make_tree(file_coverage)
        XP = root.xpath
        dirname = None
        final_row = {}

        # Goes through df row-wise, and whenever there's a change in the parent
        # of a file, adds coverage.
        for i, row in df.iterrows():
            total_num, total_den = 0, 0
            # When there's a changes in parent.
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

        final_row["FileName"] = "TOTAL"
        self.df_pruned = df.append(final_row, ignore_index=True)
        logging.debug("Coverage = {}".format(str(final_row["Coverage"])))
