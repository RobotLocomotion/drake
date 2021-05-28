"""
This provides a concrete usage of `autopybind11` to be used for Drake
symbols during development under Bazel.

Please see the neighboring README.md for more information.
"""

from contextlib import contextmanager
import argparse
import tarfile
from tempfile import TemporaryDirectory, mkdtemp
import os
import shlex
import sys
from subprocess import run, PIPE, STDOUT
import time

# TODO(eric.cousineau): Use Bazel's runfiles / Rlocation for proper resource
# finding.
from drake.tools.lint.find_data import find_data


@contextmanager
def debuggable_temporary_directory(*, dir, prefix, debug):
    if debug:
        tmp_dir = mkdtemp(prefix=prefix, dir=dir)
        yield tmp_dir
    else:
        with TemporaryDirectory(dir=dir, prefix=prefix) as tmp_dir:
            yield str(tmp_dir)


@contextmanager
def extract_archive_tempdir(archive, *, dir=None, prefix=None, debug=False):
    """
    Extracts an archive to a temporary directory.

    If `dir` is None, then it will try to resolve to $TEST_TMPDIR.
    """
    if dir is None:
        dir = os.environ.get("TEST_TMPDIR")
    with debuggable_temporary_directory(dir=dir, prefix=prefix, debug=debug) as tmp_dir:  # noqa
        with tarfile.open(archive, "r") as tar:
            tar.extractall(path=tmp_dir)
            yield str(tmp_dir)


def shlex_join(argv):
    # TODO(eric.cousineau): Replace this with `shlex.join` when we exclusively
    # use Python>=3.8.
    return " ".join(map(shlex.quote, argv))


def eprint(s):
    print(s, file=sys.stderr)


def run_autopybind11_in_bazel(argv, headers_tar, debug):
    # Since we are connecting a genrule with a proper binary, we must
    # "simulate" the genfiles directory.
    print("Running autopybind11...")
    cmd = shlex_join(argv)
    if debug:
        eprint(f"+ {cmd}")

    dt_autopybind11 = float("nan")

    with extract_archive_tempdir(headers_tar, debug=debug) as headers_dir:
        t_start = time.time()
        result = run(
            argv, cwd=headers_dir, stdout=PIPE, stderr=STDOUT,
            encoding="utf8",
            env=os.environ.copy(),
        )

    if result.returncode != 0:
        print(f"Failure for: {argv}", file=sys.stderr)
        print(result.stdout, file=sys.stderr)
    dt_autopybind11 = time.time() - t_start

    print(result.stdout)
    print()
    print(f"autopybind11 run time: {dt_autopybind11:.4f}s")
    print()

    if result.returncode != 0:
        print(f"ERROR: Failed for: {cmd}", file=sys.stderr)
        sys.exit(1)


def run_autopybind11_for_pydrake(output_dir, input_yaml, debug):
    castxml_bin = find_data("external/castxml/castxml_bin")
    if input_yaml is None:
        input_yaml = find_data(
            "tools/autopybind/bindings_to_generate.yaml")
    customization_file = find_data(
        "tools/autopybind/autopybind11_customization.yaml")
    compile_info_file = find_data("tools/autopybind/libdrake_compile_info.rsp")
    headers_tar = find_data("tools/autopybind/libdrake_headers.tar")

    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    argv = [
        sys.executable,
        "-m",
        "autopybind11",
        "--stage=2",
        "--module_name=pydrake",
        "--castxml",
        castxml_bin,
        "-o",
        output_dir,
        "-y",
        input_yaml,
        "-rs",
        compile_info_file,
        "-cg",
        customization_file,
    ]
    run_autopybind11_in_bazel(argv, headers_tar, debug)
    print(f"Wrote files to: {output_dir}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output_dir", type=str, required=True,
        help="Output all generated artifacts to this directory. Creates the "
             "directory if it does not exist.")
    parser.add_argument(
        "--input_yaml", type=str, default=None,
        help="Input YAML describing symbols for which to generate bindings. "
             "If not specified, will use version-controlled configuration "
             "file.")
    parser.add_argument(
        "--debug", action="store_true",
        help="Include verbose output and keep intermediate artifacts.")
    args = parser.parse_args()
    run_autopybind11_for_pydrake(args.output_dir, args.input_yaml, args.debug)


if __name__ == "__main__":
    main()
