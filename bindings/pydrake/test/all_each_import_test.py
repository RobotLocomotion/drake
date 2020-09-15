import multiprocessing as mp
import subprocess
import sys
import unittest


def check_module(name):
    # Run a new interpreter and ensure we can import the module in isolation.
    print(name)
    subprocess.run([sys.executable, "-c", f"import {name}"], check=True)


class Test(unittest.TestCase):
    def test_each_import(self):
        import pydrake.all

        names = []
        for name in sys.modules.keys():
            if not name.startswith("pydrake.") or "._" in name:
                continue
            if name == "pydrake.all":
                continue
            names.append(name)

        with mp.Pool(4) as pool:
            pool.map(check_module, names)
