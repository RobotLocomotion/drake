import unittest


class FlaskImportTest(unittest.TestCase):
    """Stub test class to confirm flask can be imported during tests."""

    def test_import(self):
        try:
            import flask
            can_import = True
        except ImportError:
            can_import = False

        self.assertTrue(can_import, "Cannot import flask.")
