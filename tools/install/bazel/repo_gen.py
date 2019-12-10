"""Creates the installed flavor of repo.bzl by combining repo.bzl with the
_manifest.bzl and the *.BUILD.bazel stubs, where the stubs are captured as
dictionary strings at the end of the generated repo.bzl.  (A single-file
loadable bzl file is easier to implement as part of the binary install tree.)
"""

import argparse
import os

from bazel_tools.tools.python.runfiles import runfiles


def _read(filename_in_current_package):
    # Return the string contents of the given filename in `.`.
    path = runfiles.Create().Rlocation(
        "drake/tools/install/bazel/" + filename_in_current_package)
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    # The repo_gen.bzl starts out as just repo.bzl
    repo_bzl = _read("repo.bzl")
    repo_bzl += "\n"

    # To repo.bzl we append a dict of *.bzl / *.bazel file contents.
    build_file_contents = {
        "BUILD.bazel":
            _read("drake.BUILD.bazel"),
        "bindings/pydrake/BUILD.bazel":
            _read("drake__bindings__pydrake.BUILD.bazel"),
        "common/BUILD.bazel":
            _read("drake__common.BUILD.bazel"),
        "examples/BUILD.bazel":
            _read("drake__examples.BUILD.bazel"),
        "manipulation/BUILD.bazel":
            _read("drake__manipulation.BUILD.bazel"),
    }
    repo_bzl += '_build_file_contents = {\n'
    for path, body in build_file_contents.items():
        repo_bzl += f'  "{path}": r"""\n'
        repo_bzl += body
        repo_bzl += '""",\n'
    repo_bzl += '}\n\n'

    # To repo.bzl we append the manifest.
    repo_bzl += _read("_manifest.bzl")

    # Write repo_gen.bzl.
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(repo_bzl)


if __name__ == "__main__":
    main()
