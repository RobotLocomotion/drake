import unittest


class TestDotfile(unittest.TestCase):
    def setUp(self):
        self.maxDiff = None

    def test_dotfile_consistency(self):
        # Drake's bindings/pydrake/.clang-format file should be a prologue atop
        # the root .clang-format file.
        with open(".clang-format") as f:
            root_contents = f.readlines()
        with open("bindings/pydrake/.clang-format") as f:
            bindings_contents = f.readlines()

        # The bindings file should be longer.
        self.assertGreater(len(bindings_contents), len(root_contents))
        offset = len(bindings_contents) - len(root_contents)
        assert offset > 0

        # Every line in root should appear in bindings.
        self.assertMultiLineEqual(
            "".join(root_contents),
            "".join(bindings_contents[offset:]))
