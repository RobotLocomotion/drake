from __future__ import print_function

import unittest
import sys

# Test module shim functionality.


class TestModuleShim(unittest.TestCase):
    def tearDown(self):
        # Remove the module.
        import module_shim_example as mod
        name = mod.__name__
        # There should be three references at this point:]
        # (1) sys.modules, (2) `mod`, and (3) argument to `getrefcount`.
        self.assertEqual(sys.getrefcount(mod), 3)
        del sys.modules[name]
        del mod

    def testNominal(self):
        # Test reading and writing as one would do with a normal module.
        import module_shim_example as mod
        self.assertEqual(mod.value, 1)
        mod.value += 10
        self.assertEqual(mod.value, 11)
        mod.something_new = 10
        self.assertEqual(mod.something_new, 10)
        self.assertEqual(mod.__all__, ["value", "import_type"])
        self.assertTrue(
            str(mod).startswith("<module 'module_shim_example' from"))

    def testImportDirect(self):
        # Test an import with direct access.
        import module_shim_example as mod
        self.assertEqual(mod.import_type, "direct")

    def testImportFromDirect(self):
        # Test an import via `from`.
        # N.B. This is import because `from {mod} import {var}` could end up
        # resolving `{var}` as a module.
        # @ref https://docs.python.org/3/reference/simple_stmts.html#import
        from module_shim_example import import_type
        self.assertEqual(import_type, "from_direct")


if __name__ == '__main__':
    result = unittest.main(exit=False).result
    assert result.wasSuccessful()

    # testImportFromAll:
    # Test `from x import *` since we cannot call it inside a function.
    from module_shim_example import *
    assert import_type == "from_all"
