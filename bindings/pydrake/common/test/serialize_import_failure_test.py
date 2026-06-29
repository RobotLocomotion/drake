import unittest

from pydrake.common import _binder


class TestSerializeImportFailure(unittest.TestCase):
    def test_import_exception(self):
        """Confirms that serialized types will fail-fast when they are missing
        a py::module_::import_() statement for a member field's type.
        """
        # Bar has a field of type Foo, but forgot to py::module_::import_ it.
        # This is an error.
        with self.assertRaises(ImportError) as cm:
            from pydrake.common.test.serialize_test_bar import Bar  # noqa: F401
        message = str(cm.exception)
        if _binder == "pybind11":
            self.assertIn("an import is missing", message)
            self.assertIn("drake::pydrake::test::Foo", message)
        else:
            # XXX porting This error message is no good.
            self.assertIn("error while initializing the extension", message)
