import unittest


class TestSerializeImportFailure(unittest.TestCase):

    def test_import_exception(self):
        """Confirms that serialized types will fail-fast when they are missing
        a py::module::import() statement for a member field's type.
        """
        # Bar has a field of type Foo, but forgot to py::import it.
        # This is an error.
        with self.assertRaises(ImportError) as cm:
            from pydrake.common.test.serialize_test_bar import Bar
        message = str(cm.exception)
        self.assertIn("unable to find type info", message)
        self.assertIn("drake::pydrake::test::Foo", message)
