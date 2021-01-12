"""
Regression tests for coverage.

To regenerate regression artifacts, run the following (after the test fails):

    cp \
        bazel-bin/tools/workspace/pybind11/file_coverage_test.csv \
        tools/workspace/pybind11/test/sample_header_file_coverage_expected.csv
    cp \
        bazel-bin/tools/workspace/pybind11/class_coverage_test.csv \
        tools/workspace/pybind11/test/sample_header_class_coverage_expected.csv
    cp \
        bazel-bin/tools/workspace/pybind11/sample_header_documentation_test.xml \
        tools/workspace/pybind11/test/sample_header_documentation_pybind_expected.xml
"""  # noqa

import filecmp
import os
import sys
import unittest
import xml.etree.ElementTree as ET

from drake.tools.workspace.pybind11.mkdoc import prettify


def _read(filename):
    with open(filename) as f:
        return f.read()


def _xml_normalize(elem):
    # Normalizes an XML element (e.g. consistent attribute ordering).
    new_elem = ET.Element(elem.tag)
    new_elem.attrib = dict(sorted(elem.attrib.items()))
    new_elem.text = elem.text
    for child in elem:
        new_child = _xml_normalize(child)
        new_elem.append(new_child)
    return new_elem


def _read_xml(filename):
    # Read XML to a string, normalize things like attribute ordering.
    root = ET.parse(filename).getroot()
    new_root = _xml_normalize(root)
    return prettify(new_root)


class TestLibclangParser(unittest.TestCase):
    def assert_file_equal(self, actual_file, expected_file):
        actual = _read(actual_file)
        expected = _read(expected_file)
        self.assertMultiLineEqual(
            actual,
            expected,
            f"Files are different:\n  {actual_file}\n  {expected_file}")

    def assert_xml_file_semantically_equal(self, actual_file, expected_file):
        """Read two documents and compare the re-exported string of their
        contents to normalize things like attribute ordering."""
        # N.B. This was added because attribute ordering changed in Python 3.8.
        actual = _read_xml(actual_file)
        expected = _read_xml(expected_file)
        self.maxDiff = None
        self.assertMultiLineEqual(
            actual,
            expected,
            f"Files have semantic XML differences:"
            f"\n  {actual_file}\n  {expected_file}")

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
        self.assert_xml_file_semantically_equal(
            self.get_test_file("sample_header_documentation_test.xml"),
            self.get_test_file(
                "test/sample_header_documentation_pybind_expected.xml"),
        )
