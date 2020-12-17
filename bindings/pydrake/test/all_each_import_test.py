import subprocess
import sys
import unittest

import pydrake.all


def check_module(name):
    # Run a new interpreter and ensure we can import the module in isolation.
    print(name)
    subprocess.run([sys.executable, "-c", f"import {name}"], check=True)


class Test(unittest.TestCase):
    def test_each_import(self):
        names = []
        for name in sys.modules.keys():
            if not name.startswith("pydrake.") or "._" in name:
                continue
            if name == "pydrake.all":
                continue
            names.append(name)
        self.assertGreater(len(names), 0)

        for name in sorted(names):
            check_module(name)
