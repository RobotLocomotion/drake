import filecmp
import os
import unittest


def _read(filename):
    with open(filename) as f:
        return f.read()


class TestLibclangParser(unittest.TestCase):
    def assert_file_equal(self, actual_file, expected_file):
        actual = _read(actual_file)
        expected = _read(expected_file)
        self.assertMultiLineEqual(actual, expected, actual_file)

    def get_test_file(self, relpath):
        return os.path.join("tools/workspace/pybind11", relpath)

    def test_parser(self):
        self.assert_file_equal(
            self.get_test_file("file_coverage_test.csv"),
            self.get_test_file("test/file_coverage.csv"),
        )
        self.assert_file_equal(
            self.get_test_file("class_coverage_test.csv"),
            self.get_test_file("test/class_coverage.csv"),
        )
        self.assert_file_equal(
            self.get_test_file("documentation_pybind_test.xml"),
            self.get_test_file("test/documentation_pybind.xml"),
        )
