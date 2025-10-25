from collections import namedtuple
import re
import subprocess
import sys
import unittest

# Any symbols whose name starts with one of these strings are okay.
_GOOD_SYMBOLS_PREFIX = [
    # For now, allow any typeinfo symbols.
    "_ZTI",
    "_ZTS",
    # For now, allow anything in `namespace std {}`. (We should
    # probably be checking for unwanted template arguments.)
    "_ZGVNSt7__cxx11",
    "_ZGVZNKSt8__detail",
    "_ZN9__gnu_cxx",
    "_ZNKRSt",
    "_ZNKSt",
    "_ZNSt",
    "_ZSt",
    "_ZTVNSt",
    "_ZTVSt",
    "_ZZNKSt",
    "_ZZNSt",
    "_ZZSt",
]

# Any symbols whose name contains one of these strings are okay.
_GOOD_SYMBOLS_SUBSTR = [
    # Symbols from Drake are fine.
    "N5drake",
    "NK5drake",
    "NO5drake",
    "drake_set_assertion_failure_to_throw_exception",
    # Symbols from Drake's public externals are fine.
    "N3fmt",
    "N5Eigen",
    "N6spdlog",
    "NK3fmt",
    "NK5Eigen",
    "NK6spdlog",
    # Symbols from Drake's vendored externals are fine. (It would be better
    # for performance if they could be hidden, but they are not hazardous.)
    "drake_vendor",
]

# Any symbols whose name contains one of these are undesirable, but for now
# will not cause this test to fail.
_KNOWN_BAD_SYMBOLS_SUBSTR = [
    "3tbb6detail",  # TODO(#20898): This line should be removed eventually.
    "8rules_cc2cc8runfiles",
    "Ampl",
    "BitVector128",
    "Clp",
    "Coin",
    "EventHandler",
    "FactorPointers",
    "GLEW",
    "GLXEW",
    "Idiot",
    "MessageHandler",
    "N3uWS",
    "N5ofats10any_detail",
    "Realpath",
    "WindowsError",
    "action",
    "alternativeEnvironment",
    "ampl_obj_prec",
    "boundary_sort",
    "charToStatus",
    "clp_",
    "coin",
    "ekk",
    "fileAbsPath",
    "freeArgs",
    "freeArrays",
    "getFunctionValueFromString",
    "getNorms",
    "glew",
    "innerProduct",
    "maximumAbsElement",
    "maximumIterations",
    "multiplyAdd",
    "presolve",
    "setElements",
    "setupForSolve",
    "slack_value",
    "sortOnOther",
    "wrapper",
]


class ExportedSymbolsTest(unittest.TestCase):
    def setUp(self):
        self._readelf_repair_pattern = None

    @unittest.skipIf(sys.platform == "darwin", "Ubuntu only")
    def test_exported_symbols(self):
        """Confirms that the symbols exported by libdrake.so are only:
        - Drake API (`namespace drake { ... }`).
        - Vendored externals (`namespace drake_vendor { ... }`).

        Note that many vendored externals are hidden (no exported symbols) but
        in some cases that's not possible and the symbols end up being public.

        For simplicity, we only test on Ubuntu. Hopefully, macOS is similar.
        """
        libdrake = "tools/install/libdrake/libdrake.so"
        command = ["readelf", "--wide", "--symbols"]
        output = subprocess.check_output(command + [libdrake]).decode("utf8")
        lines = output.splitlines()

        # Now we get to parse the readelf output.  The lines look like this:
        #
        # Symbol table '.dynsym' contains 58984 entries:
        #    Num:    Value          Size Type    Bind   Vis      Ndx Name
        #      0: 0000000000000000     0 NOTYPE  LOCAL  DEFAULT  UND
        #      1: 0000000000000000     0 FUNC    GLOBAL DEFAULT  UND hypot@...
        #
        # The first time we see a "Num:" header, we'll make it a namedtuple.
        # After that, each row of data will be parsed into that namedtuple.
        Row = None
        symbols = []
        for i, line in enumerate(lines):
            # Skip over useless lines.
            if not line or line.startswith("Symbol table"):
                continue
            line = self._repair_readelf_output(line)
            values = line.split()
            # Deal with the table header row.
            if line.strip().startswith("Num:"):
                column_names = tuple([x.strip(":") for x in values])
                assert column_names[-1] == "Name", line
                name_col_start = line.index("Name")
                assert name_col_start > 0, line
                if Row is None:
                    Row = namedtuple("Row", column_names, rename=True)
                else:
                    assert column_names == Row._fields
                continue
            # Skip over any lines of junk (e.g., with no symbol name).
            if len(line) == name_col_start:
                continue
            # Discard any symbol versions like "(3)" at end-of-line.
            if values[-1].endswith(")"):
                assert values[-1].startswith("("), line
                values.pop()
            # Add this line to the list of symbols.
            assert len(values) == len(Row._fields), line
            symbols.append(Row._make(values))

        # Check the symbols against our policy.
        bad_rows = [row for row in symbols if not self._is_symbol_ok(row)]
        bad_rows = sorted(bad_rows, key=lambda x: (x.Type, x.Name))

        # Report the first few errors.
        for row in bad_rows[:25]:
            print(f"{row.Type} {row.Bind} {row.Vis}")
            print(f" {self._demangle(row.Name)}")
            print(f" {row.Name}")
            print()
        self.assertEqual(len(bad_rows), 0)

    def _repair_readelf_output(self, line):
        # Mop up any "<OS specific>: %d" or "<processor specific>: # %d" output
        # that could occur in the symbol binding or type columns.
        #
        # Sometimes, instead of getting line like:
        #  990: 00000000036827b0     8 OBJECT  UNIQUE DEFAULT   ...
        #
        # Instead, we get
        #  990: 00000000036827b0     8 OBJECT  <OS specific>: 10 DEFAULT   ...
        #
        # This method performs a substitution to get rid of the unusual output,
        # so the returned result looks like this:
        #  990: 00000000036827b0     8 OBJECT  SPECIFIC DEFAULT   ...
        #
        # This type of output was noticed using the `mold` linker, see issue:
        # https://github.com/rui314/mold/issues/651
        if self._readelf_repair_pattern is None:
            self._readelf_repair_pattern = re.compile(r"<\w+ specific>: \d+")
        return self._readelf_repair_pattern.sub("SPECIFIC", line)

    @staticmethod
    def _is_symbol_ok(row):
        # Local symbols don't matter.
        if row.Bind == "LOCAL":
            return True
        # BSS start / end / etc don't matter.
        if row.Type == "NOTYPE":
            return True
        # Undefined references don't matter.
        if row.Ndx == "UND":
            return True
        name = row.Name
        for prefix in _GOOD_SYMBOLS_PREFIX:
            if name.startswith(prefix):
                return True
        for needle in _GOOD_SYMBOLS_SUBSTR:
            if needle in name:
                return True
        for needle in _KNOWN_BAD_SYMBOLS_SUBSTR:
            if needle in name:
                return True
        return False

    @staticmethod
    def _demangle(x):
        # Demangling is used only for diagnostic output, not for the allow
        # lists for symbol validation. (It's both too slow and too brittle
        # to use for the actual validation.)
        return subprocess.check_output(["c++filt"], input=x, encoding="utf-8")
