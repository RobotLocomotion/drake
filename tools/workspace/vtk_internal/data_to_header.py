# A re-implementation of upstream's vtkEncodeString.cmake. This is not a *full*
# implementation. It is sufficient for the files that we actually run through
# the conversion.

import argparse
from pathlib import Path


def _is_binary(path: Path):
    """Determines if the file at the indicated path is a binary file or not.
    The list is a strict list of the file types we know we're converting. If
    we end up converting other file types, it will require appending the
    types here; they can't be added by accident.
    """
    BINARY_EXT = [".jpg"]
    ASCII_EXT = [".glsl"]
    ext = path.suffix.lower()
    if ext in BINARY_EXT:
        return True
    elif ext in ASCII_EXT:
        return False
    else:
        raise ValueError(f"No support for converting {ext} files yet.")


def _header(constant_name: str, binary: bool):
    # Choice of unsigned char/char comes from VTK.
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


def _print_binary_as_hex(in_file):
    """Converts a binary file to a sequence of comma-separated hexadecimal byte
    values, suitable for initializing an array of bytes in C++.
    """
    all_hex = ", ".join([f"0x{b:02x}" for b in in_file.read()])
    # Don't write a single line with a billion columns. Each byte takes
    # six characters (0xAA, ). Chop off chunks to fit 80-column lines.
    width = 78  # 13 * 6
    start = 0
    while start < len(all_hex):
        print(f"  {all_hex[start : start + width].strip()}")
        start += width


def _print_conversion(source: Path):
    is_binary = _is_binary(source)
    constant_name = source.stem
    print(_header(constant_name, is_binary))
    file_code = "b" if is_binary else ""
    with open(source, f"r{file_code}") as in_file:
        if is_binary:
            _print_binary_as_hex(in_file)
        else:
            print(in_file.read())
    print(_footer(is_binary))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("in_path", type=Path, help="The source file to convert")
    args = parser.parse_args()

    _print_conversion(args.in_path)


if __name__ == "__main__":
    main()
