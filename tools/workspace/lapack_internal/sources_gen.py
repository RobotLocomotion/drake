"""Parses LAPACK's Makefiles to extract the variables that list the groupings
of source files."""

import argparse
import functools
from pathlib import Path
import re

from python import runfiles


# These are the Makefile variables we need to extract (in order).
_WANTED = (
    "DBLAS1",
    "ALLBLAS",
    "DBLAS2",
    "DBLAS3",
    "ALLAUX",
    "DZLAUX",
    "DSLASRC",
    "DLASRC",
)


@functools.cache
def _runfiles_manifest():
    return runfiles.Create()


def _slurp_file_from_runfiles(*, respath: str) -> str:
    """Returns the contents of a file from within the runfiles tree."""
    found = _runfiles_manifest().Rlocation(respath)
    assert found is not None, respath
    return Path(found).read_text(encoding="utf-8")


@functools.cache
def _lapack_files() -> set[str]:
    """Returns the set of files in the lapack_internal repository, as relative
    path strings, e.g., "BLAS/SRC/dgemm.f".
    """
    respath = "drake/tools/workspace/lapack_internal/everything.txt"
    everything = _slurp_file_from_runfiles(respath=respath)
    result = set()
    for line in everything.splitlines():
        _, relative_path = line.split("lapack_internal/")
        result.add(relative_path)
    return result


def _get_makefile_lines(*, subdir: str) -> list[str]:
    """Given a subdir in lapack_internal, returns the contents of its Makefile
    undoing the line continuations (i.e., backslashes at EOL).
    """
    text = _slurp_file_from_runfiles(
        respath=f"lapack_internal/{subdir}/Makefile")
    return text.replace("\\\n", " ").splitlines()


def _convert_makefile_value_to_sources(
        *, subdir: str, makefile_value: str) -> list[str]:
    """Given a subdir in lapack_internal and the value of one of its Makefile's
    variables (e.g., "DBLAS1 = abc.o def.o"), returns the list of source files
    that are compiled into that list of object files.
    """
    result = []
    for item in makefile_value.split():
        if "$(" in item:
            # Skip files that come from variables (we don't need them).
            continue
        if not item.endswith(".o"):
            # We only want the object files.
            continue
        stem = item[:-2]
        found = False
        for ext in (".f", ".F", ".f90", ".F90", ".mod"):
            src = f"{stem}{ext}"
            path = f"{subdir}/{src}".replace("SRC/../INSTALL/", "INSTALL/")
            if path in _lapack_files():
                result.append(src)
                found = True
                break
        if not found:
            raise RuntimeError(f"Cannot find src for {item} in {subdir}")
    return sorted(result)


def _get_vars(*, subdir: str) -> dict[str, list[str]]:
    """Given a subdir in lapack_internal, returns a dictionary mapping each
    Makefile variable name to its list of source files.
    """
    result = {}
    variable_assign_regex = re.compile('^([A-Z0-9]*) = (.*)$')
    for line in _get_makefile_lines(subdir=subdir):
        regex_match = variable_assign_regex.match(line)
        if not regex_match:
            continue
        (var, makefile_value) = regex_match.groups()
        result[var] = _convert_makefile_value_to_sources(
            subdir=subdir, makefile_value=makefile_value,
        )
    return result


def _generate_sources_bzl() -> str:
    """Returns the expected contents of our lock/sources.bzl, based on the
    current lapack_internal Makefiles.
    """
    vars = _get_vars(subdir="BLAS/SRC") | _get_vars(subdir="SRC")
    result = []
    for var in _WANTED:
        result.append(f"{var} = [")
        for src in vars[var]:
            result.append(f'    "{src}",')
        result.append("]")
        result.append("")
    result.pop(-1)
    return "\n".join(result) + "\n"


def _main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output", type=Path,
        help="Write the lockfile to the given path.")
    args = parser.parse_args()
    sources_bzl = _generate_sources_bzl()
    args.output.write_text(sources_bzl, encoding="utf-8")


_main()
