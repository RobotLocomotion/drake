import pickle
import subprocess
from subprocess import PIPE
import sys
import tempfile
import textwrap
import unittest

import pydrake.all
from pydrake.common import temp_directory


class TestAllEachImport(unittest.TestCase):

    def setUp(self):
        self._expected_non_native_modules = [
            # A standalone 'import pydrake' should not trigger native code.
            "pydrake",
            # Another example of a module we'd want to be non-native would be
            # pydrake.lcmtypes, but we don't have such a module (yet).
        ]
        self._temp_dir = temp_directory()

    def test_each_import(self):
        """For all known pydrake modules, checks that they can be imported on
        their own, one by one.
        """
        names = []
        for name in sys.modules.keys():
            if "._" in name:
                # Private module.
                continue
            if name == "pydrake.all":
                # Already tested via this process's import statement.
                continue
            if name == "pydrake" or name.startswith("pydrake."):
                names.append(name)
        self.assertGreater(len(names), 0)

        for name in sorted(names):
            with self.subTest(name=name):
                self._check_module(name)

        for name in self._expected_non_native_modules:
            self.assertIn(name, names)

    def _check_module(self, name):
        """Runs a new interpreter to prove that we can import the module in
        isolation. Modulo a few allowed corner-cases, also checks that the
        pydrake.common module has also been imported by side-effect (and so,
        that it's important native-code bootstrapping logic is in effect).
        """
        temp_filename = f"{self._temp_dir}/has_common_{name}"
        script = textwrap.dedent(f"""\
            import {name}
            import pickle, sys
            has_common = bool("pydrake.common" in sys.modules)
            with open("{temp_filename}", "wb") as f:
                pickle.dump(has_common, f)
        """)
        subprocess.run([sys.executable, "-c", script], check=True)

        # Parse the output.
        with open(temp_filename, "rb") as f:
            has_common = pickle.load(f)
        self.assertIsNotNone(has_common)

        # Check for required presence / absence of native code.
        if name in self._expected_non_native_modules:
            self.assertFalse(
                has_common,
                f"The module {name} is not supposed to induce a load-time"
                " dependency on pydrake.common, but somehow it did.")
        else:
            self.assertTrue(
                has_common,
                f"The module {name} was expected to 'import pydrake.common'"
                " to force native code bootstrapping, but it did not. If"
                " the module does not contain native code, then add it to"
                " list of expected_non_native_modules. If the module does"
                " contain native code, then add this line near the start of"
                " the PYBIND11_MODULE stanza in its cc file: "
                " py::module::import(\"pydrake.common\");")
