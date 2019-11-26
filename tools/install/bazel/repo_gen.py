"""Create the installed flavor of repo.bzl by combining repo.bzl and the
*.BUILD.bazel stubs, where the stubs are captured as dictionary strings
inside the generated repo.bzl.  A single-file loadable bzl file is easier
to use as part of the binary install tree.
"""

import argparse
import os

from bazel_tools.tools.python.runfiles import runfiles


def _read(filename_in_current_package):
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

    # To repo.bzl we will append a dict of BUILD file contents.
    build_file_contents = {
        "BUILD.bazel":
            _read("drake.BUILD.bazel"),
        "bindings/pydrake/BUILD.bazel":
            _read("drake__bindings__pydrake.BUILD.bazel"),
    }
    repo_bzl += "_build_file_contents = {\n"
    for path, body in build_file_contents.items():
        repo_bzl += f'  "{path}": r"""\n'
        repo_bzl += body
        repo_bzl += '""",\n'
    repo_bzl += "}\n"

    # Write repo_gen.bzl.
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(repo_bzl)


if __name__ == "__main__":
    main()
