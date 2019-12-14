"""Creates the installed flavor of repo.bzl by combining repo_template.bzl with
the manifest.bzl and the *.BUILD.bazel stubs.  See README.md for details.
"""

import argparse
import os

from bazel_tools.tools.python.runfiles import runfiles


def _read(pathname):
    # Returns the string contents of the given pathname.
    with open(pathname, "r", encoding="utf-8") as f:
        return f.read()


def _demangle_build_file_name(pathname):
    # Turns, e.g., drake__examples.BUILD.bazel in examples/BUILD.bazel.
    result = os.path.basename(pathname)
    result = result.replace("__", "/").replace(".BUILD.bazel", "/BUILD.bazel")
    assert result.startswith("drake/")
    result = result[len("drake/"):]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--repo_template", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--build_file", action="append", dest="build_files")
    args = parser.parse_args()

    # This will be the new value for _BUILD_FILE_CONTENTS.
    build_file_contents = dict([
        (_demangle_build_file_name(pathname), _read(pathname))
        for pathname in args.build_files
    ])

    # This will be the new value for _MANIFEST.
    manifest_contents = _read(args.manifest)

    # The repo.bzl output is based on repo_template.bzl, with a few
    # programmatic alterations.
    repo_bzl = []
    for line in _read(args.repo_template).splitlines():
        # Strip template-specific comment lines.
        if line.startswith("# * #"):
            continue
        # Replace _BUILD_FILE_CONTENTS = ...
        if line.startswith("_BUILD_FILE_CONTENTS"):
            assert build_file_contents is not None
            repo_bzl.append("_BUILD_FILE_CONTENTS = {")
            for path, body in build_file_contents.items():
                repo_bzl.append(f'  "{path}": r"""')
                repo_bzl.append(body)
                repo_bzl.append('""",')
            repo_bzl.append("}")
            repo_bzl.append("")
            build_file_contents = None
            continue
        # Replace _MANIFEST = ...
        if line.startswith("_MANIFEST"):
            # The repo.bzl has a leading underscore on _MANIFEST to mark it
            # private.  In manifest.bzl, it uses MANIFEST with no underscore
            # because it is public.  We add in the underscore here.
            assert manifest_contents.startswith("MANIFEST")
            repo_bzl.append("_" + manifest_contents)
            manifest_contents = None
            continue
        # Otherwise, retain the line as-is.
        repo_bzl.append(line)

    # Make sure that we matched both lines.
    assert build_file_contents is None, "Failed to match _BUILD_FILE_CONTENTS"
    assert manifest_contents is None, "Failed to match _MANIFEST"

    # Write repo.bzl.
    with open(args.output, "w", encoding="utf-8") as f:
        f.write("\n".join(repo_bzl))


if __name__ == "__main__":
    main()
