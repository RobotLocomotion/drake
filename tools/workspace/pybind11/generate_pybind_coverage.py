import argparse

from drake.tools.workspace.pybind11.pybind_coverage_libclang_parser import (
    get_docstrings_from_bindings,
)
from drake.tools.workspace.pybind11.pybind_coverage_xml_parser import (
    ClassCoverage,
    FileCoverage,
)


def parse_arguments():
    parser = argparse.ArgumentParser(description="Generate pybind11 coverage")
    parser.add_argument("--xml_docstrings", help="The XML file created during "
                        "generation of docstrings", type=str, required=True)
    parser.add_argument("--class_coverage", help="A csv file for writing the "
                        "coverage of classes", type=str, required=True)
    parser.add_argument("--file_coverage", help="A csv file for writing the "
                        "file-wise coverage", type=str, required=True)
    parser.add_argument("--pybind_doc_variable", help="The name of the "
                        "documentation variable, defaults to 'pydrake_doc'",
                        type=str, required=False)
    parser.add_argument("pybind_source_files", nargs='+')
    args = parser.parse_args()

    return args


def main():
    args = parse_arguments()
    pybind_strings = get_docstrings_from_bindings(
            args.pybind_source_files, args.pybind_doc_variable)

    class_coverage = ClassCoverage(
            args.xml_docstrings, pybind_strings, args.class_coverage)
    class_coverage.write_coverage()

    file_coverage = FileCoverage(
        args.xml_docstrings, pybind_strings, args.file_coverage)
    file_coverage.write_coverage()


if __name__ == "__main__":
    main()
