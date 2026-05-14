from collections import namedtuple
import re
import subprocess
import unittest

from python import runfiles

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

LIBDRAKE = "tools/install/libdrake/libdrake.so"


def _is_known_bad_ctor_or_dtor(*, filename, function_name):
    is_snake_case = filename == filename.lower()
    if filename.startswith("vtk") and not is_snake_case:
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if filename in ["Singletons.cxx", "Token.cxx", "vtk_opengl_init.cc"]:
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("drake_vendor::vtk"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("vtksys::"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("vtktoken::"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if filename == "Console.cc":
        # TODO(#24446) Patch libsdformat to fix this.
        return True
    if filename == "meshcat.cc":
        # TODO(#24446) Fix this somehow.
        return True
    if filename == "impl.cpp":
        # TODO(#24447) Fix VTK to remove globals.
        return True
    return False


class ExportedSymbolsTest(unittest.TestCase):
    def setUp(self):
        self._readelf_repair_pattern = None

    def test_exported_symbols_and_global_constructors(self):
        """Confirms that the symbols exported by libdrake.so are only:
        - Drake API (`namespace drake { ... }`).
        - Vendored externals (`namespace drake_vendor { ... }`).

        Note that many vendored externals are hidden (no exported symbols) but
        in some cases that's not possible and the symbols end up being public.

        Also checks that there are no global constructors or destructors.
        """
        command = ["readelf", "--wide", "--symbols"]
        output = subprocess.check_output(command + [LIBDRAKE]).decode("utf8")
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
        if bad_rows:
            print("======== Incorrectly namespaced symbol names ========")
            print()
            for row in bad_rows[:25]:
                print(f"{row.Type} {row.Bind} {row.Vis}")
                print(f" {self._demangle(row.Name)}")
                print(f" {row.Name}")
                print()
        self.assertEqual(len(bad_rows), 0)

        # Check for static initializers and/or destructors.
        ctors, dtors = self._global_ctors_dtors(symbols)
        if ctors:
            print("======== Disallowed global constructors ========")
            print()
            for function_name in ctors:
                print(f" {function_name}")
                for filename in ctors[function_name]:
                    print(f"  called by {filename}")
                print()
        if dtors:
            print("======== Disallowed global destructors ========")
            print()
            for function_name in dtors:
                print(f" {function_name}")
                for filename in dtors[function_name]:
                    print(f"  called by {filename}")
                print()
        self.assertEqual(len(ctors), 0)
        self.assertEqual(len(dtors), 0)

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

    @staticmethod
    def _global_ctors_dtors(symbols):
        """Returns (ctors, dtors) where each is a dict mapping the disallowed
        constructor or destructor to the list of filename(s) that call it."""
        ctors = {}
        dtors = {}

        # Find symbols (if any) where the compiler puts (1) calls to global
        # constructors and (2) calls to `atexit` to register global destructors.
        bad_symbols = [
            row for row in symbols if row.Name.startswith("_GLOBAL__sub")
        ]
        if not bad_symbols:
            return ctors, dtors

        # Disassemble the sections (so we can look for calls). We need to use
        # llvm objdump instead of Ubuntu objdump because Ubuntu is >100x slower.
        bad_symbol_names = [symbol.Name for symbol in bad_symbols]
        manifest = runfiles.Create()
        objdump_all = subprocess.check_output(
            [
                manifest.Rlocation("llvm/bin/llvm-objdump"),
                LIBDRAKE,
                "-Mintel",
                "--no-addresses",
                "--no-show-raw-insn",
                "--demangle",
                f"--disassemble-symbols={','.join(bad_symbol_names)}",
            ],
            text=True,
        ).splitlines()

        # Loop over the bad symbols and find (1) functions that are directly
        # called (which are therefore invalid constructor calls), and (2)
        # function pointers that are passed to `atexit` (which are therefore
        # invalid destructor calls).
        for symbol in bad_symbols:
            filename = symbol.Name.removeprefix("_GLOBAL__sub_I_")
            ctor_calls, dtor_calls = ExportedSymbolsTest._ctor_dtor_calls(
                symbol=symbol,
                objdump_all=objdump_all,
            )
            if not ctor_calls and not dtor_calls:
                # The constructor got inlined, so we couldn't identify any
                # function names as part of a `call` instruction. Be sure to
                # still report a non-specific error.
                ctor_calls = ["unknown constructor"]
            for function_name in ctor_calls:
                is_known_bad = _is_known_bad_ctor_or_dtor(
                    filename=filename,
                    function_name=function_name,
                )
                if not is_known_bad:
                    ctors.setdefault(function_name, []).append(filename)
            for function_name in dtor_calls:
                is_known_bad = _is_known_bad_ctor_or_dtor(
                    filename=filename,
                    function_name=function_name,
                )
                if not is_known_bad:
                    dtors.setdefault(function_name, []).append(filename)
        return ctors, dtors

    @staticmethod
    def _ctor_dtor_calls(*, symbol, objdump_all):
        """Returns a pair of lists (ctor_calls, dtor_calls). Each list contains
        function names that are called by the given "__GLOBAL_sub_..." `symbol`,
        i.e., functions called during static initialization and destruction.
        """
        ctor_calls = []
        dtor_calls = []

        # Find the objdump output for our symbol.
        i = objdump_all.index(f"<{symbol.Name}>:") + 1
        instructions = []
        while i < len(objdump_all):
            line = objdump_all[i]
            if not line.startswith(" "):
                # The decompiled code is all indented by whitespace. A non-
                # indented line indicates the start of the next function.
                break
            instructions.append(line.lstrip())
            i += 1

        # Remove useless padding at the end of the function.
        while instructions[-1] == "int3":
            del instructions[-1]

        # Find calls to functions. These are the global constructor calls,
        # unless the called function is `atexit` in which case it's a global
        # destructor registration.
        atexit_indices = []
        for i, instruction in enumerate(instructions):
            if instruction.startswith("call\t"):
                function_name = instruction.split("<", maxsplit=1)[1][:-1]
                if function_name == "__cxa_atexit@plt":
                    atexit_indices.append(i)
                else:
                    ctor_calls.append(function_name)

        # Also notice tail calls to `atexit`.
        if instructions[-1].startswith("jmp\t"):
            atexit_indices.append(len(instructions) - 1)

        # For calls to `atexit`, figure out what the `rdi` argument is pointing
        # to. These are the global destructor registrations. These look like:
        #
        # lea	rdi, [rip + 0x...] # 0x... <function_name>
        # ...
        # call	0x... <__cxa_atexit@plt>
        max_lookback = 6  # At most this many instructions from lea-rdi to call.
        for i in atexit_indices:
            for lookback in range(1, max_lookback + 1):
                prior = i - lookback
                if prior < 0:
                    continue
                prior_instruction = instructions[prior]
                if not prior_instruction.startswith("lea\trdi"):
                    continue
                function_name = prior_instruction.split("<", maxsplit=1)[1][:-1]
                dtor_calls.append(function_name)
                break
            else:
                # Could not find the `lea rdi, ...` instruction.
                dtor_calls.append("unknown destructor")

        return ctor_calls, dtor_calls
