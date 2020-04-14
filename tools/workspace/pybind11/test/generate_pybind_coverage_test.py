import filecmp
import os
import sys
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
            self.get_test_file(
                "test/sample_header_file_coverage_expected.csv"),
        )
        self.assert_file_equal(
            self.get_test_file("class_coverage_test.csv"),
            self.get_test_file(
                "test/sample_header_class_coverage_expected.csv"),
        )
        if sys.version_info[:2] >= (3, 8):
            self.assert_file_equal(
                self.get_test_file("sample_header_documentation_test.xml"),
                self.get_test_file(
                    "test/sample_header_documentation_pybind_expected_python_3.8.xml"  # noqa
                ),
            )
        else:
            self.assert_file_equal(
                self.get_test_file("sample_header_documentation_test.xml"),
                self.get_test_file(
                    "test/sample_header_documentation_pybind_expected.xml"),
            )
