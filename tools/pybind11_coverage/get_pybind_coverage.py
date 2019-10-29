import parse_pybind_xml_docstrings
import argparse
import libclang_parser


def parse_arguments():
    parser = argparse.ArgumentParser(description="Generate pybind11 coverage")
    parser.add_argument("--xml_docstrings", help="The XML file created during "
                        "generation of docstrings", type=str, required=True)
    parser.add_argument("--class_coverage", help="A csv file for writing the "
                        "coverage of classes", type=str, required=True)
    parser.add_argument("--file_coverage", help="A csv file for writing the "
                        "file-wise coverage", type=str, required=True)
    parser.add_argument("pybind_source_files", nargs='+')
    args = parser.parse_args()

    return args


def main():
    args = parse_arguments()
    pybind_strings = \
        libclang_parser.get_docstring_for_bindings(args.pybind_source_files)

    parse_pybind_xml_docstrings.get_file_and_class_coverage(
            args.xml_docstrings, args.file_coverage,
            args.class_coverage, pybind_strings)


if __name__ == "__main__":
    main()
