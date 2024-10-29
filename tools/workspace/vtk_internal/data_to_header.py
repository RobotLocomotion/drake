# A re-implementation of upstream's vtkEncodeString.cmake. This is not a *full*
# implementation. It is sufficient for the files that we actually run through
# the conversion.

import argparse
from pathlib import Path


def _header(constant_name: str, binary: bool):
    byte_type = "unsigned char" if binary else "char"
    opening = "{" if binary else 'R"vtkdrakeinternal('
    return f"""
#pragma once
VTK_ABI_NAMESPACE_BEGIN
constexpr {byte_type} {constant_name}[] = {opening}"""


def _footer(binary: bool):
    closing = "};" if binary else ')vtkdrakeinternal";'
    return f"""{closing}
VTK_ABI_NAMESPACE_END
"""


def _write_binary_as_hex(in_file):
    """Converts a binary file to a sequence of comma-separated hexadecimal byte
    values, suitable for initializing an array of bytes in C++.
    """
    all_hex = ', '.join([f'0x{b:02x}' for b in in_file.read()])
    # Don't write a single line with a billion columns. Each byte takes
    # six characters (0xAA, ). Chop off chunks to fit 80-column lines.
    width = 78  # 13 * 6
    start = 0
    while start < len(all_hex):
        print(f"  {all_hex[start:start+78].strip()}")
        start += width


def _print_conversion(source: Path, binary: bool):
    constant_name = source.stem
    file_code = "b" if binary else ""
    print(_header(constant_name, binary))
    with open(source, f"r{file_code}") as in_file:
        if binary:
            _write_binary_as_hex(in_file)
        else:
            print(in_file.read())
    print(_footer(binary))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "in_path", type=Path, help="The source file to convert")
    parser.add_argument(
        "--binary", action='store_true',
        help="If true, the input file will be interpreted as binary")
    args = parser.parse_args()

    _print_conversion(args.in_path, args.binary)


if __name__ == '__main__':
    main()
