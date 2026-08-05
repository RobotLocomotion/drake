import bisect
import os
import struct
import subprocess
import unittest

import lief
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
    if function_name.startswith("vtk::drake_vendor::"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("vtkpugixml::"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("vtksys::"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("vtktoken::"):
        # TODO(#24447) Fix VTK to remove globals.
        return True
    if function_name.startswith("__static_initialization_and_destruction_0"):
        # This appears to be a secondary effect of other globals. Once we've
        # fixed all of the other bad ctors and dtors, we can circle back here
        # to see if this suppression can also be removed.
        return True
    return False


class ExportedSymbolsTest(unittest.TestCase):
    def test_everything(self):
        """Checks several libdrake.so correctness conditions one after another.
        (Parsing libdrake.so is expensive so we do it once and then use it for
        multiple checks.)

        (1) Confirms that the symbols exported by libdrake.so are only:
        - Drake API (`namespace drake { ... }`).
        - Vendored externals (`namespace drake_vendor { ... }`).

        Note that many vendored externals are hidden (no exported symbols) but
        in some cases that's not possible and the symbols end up being public.

        (2) Checks that there are no global constructors or destructors.

        (3) Checks that there are no direct calls to __cxa_atexit (a symptom
        of function-local static destructors).
        """
        binary = lief.parse(LIBDRAKE)
        symbols = list(binary.symbols) + list(binary.dynamic_symbols)

        # Check the symbols against our policy.
        bad_symbols = [
            symbol for symbol in symbols if not self._is_symbol_ok(symbol)
        ]
        bad_symbols = sorted(
            bad_symbols,
            key=lambda x: (x.type.name, x.name),
        )

        # Report the first few errors.
        if bad_symbols:
            print("======== Incorrectly namespaced symbol names ========")
            print()
            for symbol in bad_symbols[:25]:
                print(f"{symbol.type} {symbol.binding} {symbol.visibility}")
                print(f" {self._demangle(symbol.name)}")
                print(f" {symbol.name}")
                print()
        self.assertEqual(len(bad_symbols), 0)

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

        # Check for direct callers of __cxa_atexit via relocation byte-scanning.
        atexit_callers = self._find_cxa_atexit_callers(binary)
        if atexit_callers:
            print("======== Disallowed calls to __cxa_atexit ========")
            print()
            for function_name in atexit_callers:
                print(f" {function_name}")
        self.assertEqual(len(atexit_callers), 0)

    @staticmethod
    def _is_symbol_ok(symbol):
        # Local symbols don't matter.
        if symbol.binding == lief.ELF.Symbol.BINDING.LOCAL:
            return True
        # BSS start / end / etc don't matter.
        if symbol.type == lief.ELF.Symbol.TYPE.NOTYPE:
            return True
        # Undefined references don't matter.
        if getattr(symbol, "is_undefined", False) or symbol.shndx == 0:
            return True
        name = symbol.name
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
            symbol
            for symbol in symbols
            if symbol.name.startswith("_GLOBAL__sub")
        ]
        if not bad_symbols:
            return ctors, dtors

        # Disassemble the sections (so we can look for calls). We need to use
        # llvm objdump instead of Ubuntu objdump because Ubuntu is >100x slower.
        bad_symbol_names = [symbol.name for symbol in bad_symbols]
        manifest = runfiles.Create()
        objdump_all = subprocess.check_output(
            [
                manifest.Rlocation(os.environ["LLVM_OBJDUMP_RLOCATIONPATH"]),
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
            filename = symbol.name.removeprefix("_GLOBAL__sub_I_")
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
        i = objdump_all.index(f"<{symbol.name}>:") + 1
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

    @staticmethod
    def _find_cxa_atexit_callers(binary):
        """Returns a list of (demangled) function names that call __cxa_atexit,
        except for those _is_known_bad_ctor_or_dtor already knows are bad.
        """
        # Locate the PLT virtual memory address ("VMA") for __cxa_atexit.
        for i, reloc in enumerate(binary.pltgot_relocations):
            if reloc.has_symbol and reloc.symbol.name == "__cxa_atexit":
                # The PLT has a 16-byte header and then 16 bytes per relocation,
                # so the i'th relocation's VMA is at (base + 16 + 16*i).
                plt_vma = binary.get_section(".plt").virtual_address
                atexit_vma = plt_vma + 16 + (16 * i)
                break
        else:
            # No relocations for __cxa_atexit means that nobody called it.
            return []

        # Scan the text section (code) for x86_64 call opcodes (0xE8), finding
        # all calls to the __cxa_atexit PLT.
        atexit_call_vmas = []
        # The loop variable "i" is an index into text_bytes.
        text_section = binary.get_section(".text")
        text_bytes = bytes(text_section.content)
        i = 0
        while True:
            # Find what *could* be a call opcode and extract its instruction-
            # relative target address (the signed 32-bit number that follows
            # the opcode).
            i = text_bytes.find(b"\xe8", i)
            if i < 0 or i + 5 > len(text_bytes):
                # We've reached the end of text_bytes.
                break
            target_offset = struct.unpack("<i", text_bytes[i + 1 : i + 5])[0]
            instruction_vma = text_section.virtual_address + i
            target_vma = instruction_vma + 5 + target_offset
            if target_vma == atexit_vma:
                atexit_call_vmas.append(instruction_vma)
            # We don't know whether `i` was an actual call opcode, because we
            # don't know where instructions start and stop. Advance our cursor
            # by only 1 byte (not 5 bytes), in case it was a false positive.
            i += 1
        # If __cxa_atexit has a relocation, it should have been used somewhere.
        assert atexit_call_vmas

        # Map addresses of atexit calls back to names of the calling functions.
        # We'll build a table of (vma_start, vma_end, name) for binary search.
        func_table = [
            (symbol.value, symbol.value + symbol.size, symbol.name)
            for symbol in list(binary.symbols) + list(binary.dynamic_symbols)
            if symbol.is_function and symbol.size > 0
        ]
        func_table.sort(key=lambda x: x[0])
        vma_starts = [x[0] for x in func_table]

        def lookup_symbol(vma):
            i = bisect.bisect_right(vma_starts, vma) - 1
            if i >= 0:
                vma_start, vma_end, name = func_table[i]
                if vma_start <= vma < vma_end:
                    return name
            return f"unknown_symbol@0x{vma:x}"

        bad_symbol_names = sorted(
            set([lookup_symbol(vma) for vma in atexit_call_vmas])
        )

        # Apply the "known bad" allow-list.
        failures = []
        for name in bad_symbol_names:
            if name.startswith("_GLOBAL__sub_I_"):
                # The "_global_ctors_dtors" test function (above) already
                # checked this symbol (with more details than we have here),
                # so we'll skip it here to avoid duplicate reports.
                continue
            function_name = ExportedSymbolsTest._demangle(name).strip()
            if not _is_known_bad_ctor_or_dtor(
                filename="",
                function_name=function_name,
            ):
                failures.append(function_name)

        return failures
