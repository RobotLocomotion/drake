"""
Upgrades the lockfile for OpenUSD.
"""

import argparse
import functools
from pathlib import Path
import re
import subprocess
import tempfile

from python import runfiles

# This is the full list of OpenUSD libraries that Drake cares about.
SUBDIRS = [
    "pxr/base/arch",
    "pxr/base/gf",
    "pxr/base/js",
    "pxr/base/pegtl",
    "pxr/base/plug",
    "pxr/base/tf",
    "pxr/base/trace",
    "pxr/base/ts",
    "pxr/base/vt",
    "pxr/base/work",
    "pxr/usd/ar",
    "pxr/usd/kind",
    "pxr/usd/ndr",
    "pxr/usd/pcp",
    "pxr/usd/sdf",
    "pxr/usd/sdr",
    "pxr/usd/usd",
    "pxr/usd/usdGeom",
    "pxr/usd/usdPhysics",
    "pxr/usd/usdShade",
    "pxr/usd/usdUtils",
]

# This code block is crafted to work with the code scraped from
# OpenUSD/cmake/macros/Public.cmake. It is the replacement code for
# pxr_library(), that instead of building the library, reports selected
# arguments. If any of the parsed arguments reported here go missing or change
# spelling, this block will need to be updated. See _compose_cmake_prelude().
PXR_LIBRARY_FUNCTION_STUB_EPILOG = """
      # This order of keys matches the traditional order of the Drake
      # lock file.
      message("NAME = ${NAME}")
      message("INCLUDE_SCHEMA_FILES = ${args_INCLUDE_SCHEMA_FILES}")
      message("LIBRARIES = ${args_LIBRARIES}")
      message("PUBLIC_CLASSES = ${args_PUBLIC_CLASSES}")
      message("PUBLIC_HEADERS = ${args_PUBLIC_HEADERS}")
      message("PRIVATE_CLASSES = ${args_PRIVATE_CLASSES}")
      message("PRIVATE_HEADERS = ${args_PRIVATE_HEADERS}")
      message("CPPFILES = ${args_CPPFILES}")
endfunction()
"""


def _slurp_file_from_runfiles(path: str) -> str:
    """Returns the contents of a file from within the runfiles tree."""
    manifest = runfiles.Create()
    found = manifest.Rlocation(path)
    assert found is not None, path
    return Path(found).read_text(encoding="utf-8")


def _expand_stub(function: str) -> str:
    """Returns cmake source code of a no-op function, with the given name."""
    return f"""
function({function})
endfunction()
"""


@functools.cache
def _compose_cmake_prelude() -> str:
    """Scrapes the contents of the cmake/macros/Public.cmake file from OpenUSD.
    Compose a prelude of redefined (mostly stubbed-out) functions to use when
    interpreting each library's CMakeLists.txt file.
    """
    # Slurp the upstream macros file.
    public = _slurp_file_from_runfiles(
        "openusd_internal/cmake/macros/Public.cmake")

    # Build the list of functions to provide as no-op stubs.
    functions = re.findall(r'.*\bfunction\b *\( *([^) ]+)[) ]', public)
    assert functions
    functions.remove("pxr_library")

    # Make source code stubs for all those functions.
    stubs = "\n\n".join([_expand_stub(function) for function in functions])

    # Scrape out the pxr_library argument parsing. We need the text from the
    # start of the pxr_library function definition to the end of the invocation
    # of cmake_parse_arguments.
    match = re.search(
        r'.*(\bfunction\b *\( *pxr_library.*?'
        r'cmake_parse_arguments *\([^)]*?\))',
        public, flags=re.DOTALL)
    assert match
    pxr_library_parsing = match.group(1)

    # Combine pxr library parsing with our custom code, and the function stubs.
    return "\n\n".join([pxr_library_parsing, PXR_LIBRARY_FUNCTION_STUB_EPILOG,
                        stubs])


def _interpret(cmake: str) -> dict[str, str | list[str]]:
    """Uses the cmake interpreter to extract the arguments of the
    pxr_library() call in the given cmake source code string."""
    prelude = _compose_cmake_prelude()
    script = "\n\n".join([prelude, cmake])
    with tempfile.NamedTemporaryFile(
            prefix="drake_openusd_internal_print_pxr_library_args_") as f:
        f.write(script.encode("utf8"))
        f.flush()
        command = ["cmake", "-P", f.name]
        output = subprocess.check_output(
            command, stderr=subprocess.STDOUT).decode("utf8")

    result = dict()
    for line in output.splitlines():
        if " = " not in line:
            continue
        k, v = line.split(" = ")
        if k == "NAME":
            result[k] = v
        else:
            vlist = [e for e in v.split(";") if e]
            result[k] = vlist
    return result


def _parse_generated_schema(classes_txt: str) -> list[str]:
    """Given the contents of a generatedSchema.classes.txt file, returns the
    list of 'Public Classes'.
    """
    content = {}
    section = None
    for line in classes_txt.splitlines():
        if line.startswith("# "):
            section = line[2:]
            continue
        if not section:
            continue
        if line:
            items = content.get(section, [])
            items.append(line)
            content[section] = items
    return content.get("Public Classes", [])


def _extract(subdir: str) -> dict[str, str | list[str]]:
    """Extracts the pxr_library() call from the given subdir's CMakeLists.txt
    and returns its arguments as a more Pythonic data structure. (Refer to the
    `result` dict sanity checking inline below, for details.)
    """
    cmake = _slurp_file_from_runfiles(
        f"openusd_internal/{subdir}/CMakeLists.txt")
    result = _interpret(cmake)

    # The CMakeLists.txt does not necessarily list all public classes. When it
    # says INCLUDE_SCHEMA_FILES then there is a sidecar file with more classes.
    has_schema_files = result.pop("INCLUDE_SCHEMA_FILES")[0] == 'TRUE'
    if has_schema_files:
        public_classes = result["PUBLIC_CLASSES"]
        classes_txt = _slurp_file_from_runfiles(
            f"openusd_internal/{subdir}/generatedSchema.classes.txt")
        more_public_classes = _parse_generated_schema(classes_txt)
        public_classes.extend(more_public_classes)
        result["PUBLIC_CLASSES"] = sorted(public_classes)

    # Result dict must contain these keys.
    assert set(result.keys()) == {"NAME", "PUBLIC_CLASSES", "PUBLIC_HEADERS",
                                  "PRIVATE_CLASSES", "PRIVATE_HEADERS",
                                  "CPPFILES", "LIBRARIES"}
    # NAME has a string-type value, all the rest are lists.
    for key in result.keys():
        if key == "NAME":
            required_type = str
        else:
            required_type = list
        assert isinstance(result[key], required_type)
    return result


def _generate() -> str:
    """Returns the expected contents of the lockfile (lock/files.bzl)."""
    lines = [
        "# This file is automatically generated by upgrade.py.",
    ]
    lines.append("FILES = {")
    for subdir in SUBDIRS:
        lines.append(f'    "{subdir}": {{')
        extracted = _extract(subdir)
        for name, value in extracted.items():
            if isinstance(value, str):
                lines.append(f'        "{name}": "{value}",')
                continue
            assert isinstance(value, list)
            if len(value) == 0:
                lines.append(f'        "{name}": [],')
                continue
            lines.append(f'        "{name}": [')
            for item in value:
                lines.append(f'            "{item}",')
            lines.append(f'        ],')
        lines.append(f'    }},')
    lines.append("}")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(
        prog="upgrade", description=__doc__)
    parser.add_argument(
        "--relock", action="store_true",
        help="Overwrite the lockfile in the source tree.")
    parser.add_argument(
        "--output", type=Path,
        help="Write the lockfile to the given path.")
    args = parser.parse_args()
    assert args.relock ^ (args.output is not None)
    if args.relock:
        output = Path(__file__).resolve().parent / "lock/files.bzl"
    else:
        output = args.output
    content = _generate()
    output.write_text(content, encoding="utf-8")


assert __name__ == "__main__"
main()
