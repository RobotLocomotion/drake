"""Run Google's buildifier tool to fix, check, or fix-diff BUILD files.

All arguments except "--all" are passed through to Google's tool.  As with the
Google tool, the default mode is "-mode=fix".  In "-mode=check", we promote
lint errors to a non-zero exitcode.
"""

import os
from pathlib import Path
import re
import subprocess
from subprocess import PIPE, STDOUT, Popen
import sys

from python import runfiles

from tools.lint.util import find_all_sources


def _make_buildifier_command():
    """Returns a list starting with the buildifier executable, followed by any
    required default arguments."""
    manifest = runfiles.Create()
    resource_path = "buildifier_prebuilt/buildifier/buildifier"
    buildifier = manifest.Rlocation(resource_path)
    tables = manifest.Rlocation("drake/tools/lint/buildifier-tables.json")
    assert buildifier is not None
    assert tables is not None
    buildifier = Path(buildifier).absolute()
    tables = Path(tables).absolute()
    return [buildifier, f"-add_tables={tables}"]


def _help(command):
    """Perform the --help operation (display output) and return an exitcode."""
    process = Popen(command, stdout=PIPE, stderr=STDOUT, encoding="utf-8")
    stdout, _ = process.communicate()
    lines = stdout.splitlines()
    # Edit the first line to allow "--all" as a disjunction from "files...",
    # and make one or the other required.
    head = re.sub(r"\[(files\.\.\.)\]", r"<\1 | --all>", lines.pop(0))
    for line in [head] + lines:
        print(line)
    print("")
    print("=== Drake-specific additions ===")
    print("")
    print("If the --all flag is given, buildifier operates on every BUILD,")
    print("*.BUILD, *.bazel, and *.bzl file in the tree except third_party.")
    print("")
    print("Without '--all', 'files...' are required; stdin cannot be used.")
    return process.returncode


def _find_buildifier_sources():
    """Return a list of all filenames to be covered by buildifier."""
    workspace, sources_relpath = find_all_sources()
    exact_filenames = ["BUILD", "WORKSPACE"]
    extensions = ["bazel", "bzl", "BUILD"]
    return workspace, [
        os.path.join(workspace, relpath)
        for relpath in sources_relpath
        if os.path.splitext(relpath)[1][1:] in extensions
        or os.path.basename(relpath) in exact_filenames
    ]


def _passes_check_mode(args):
    """The `args` list should be as per subprocess.check_call.  Returns True
    iff builfidier runs with exitcode 0 and no output, or else returns False
    iff reformat is needed, or else raises an exception.
    """
    try:
        output = subprocess.check_output(args)
        return len(output) == 0
    except subprocess.CalledProcessError as e:
        # https://github.com/bazelbuild/buildtools/blob/1a7c0ec10697afcb87af8a09f12c3f9b9ca56fb2/buildifier/buildifier.go#L227
        REFORMAT_IS_NEEDED = 4
        if e.returncode == REFORMAT_IS_NEEDED:
            return False
        raise e


def main():
    # Slice out our overlay command-line argument "--all".
    argv = sys.argv[1:]
    find_all = False
    if "--all" in argv:
        find_all = True
        argv.remove("--all")

    # Find the wrapped tool.
    tool_cmds = _make_buildifier_command()

    # Process --help.
    if "--help" in argv or "-help" in argv:
        return _help(tool_cmds + argv)

    # Process --all.
    has_files = len([x for x in argv if not x.startswith("-")]) > 0
    if find_all and has_files:
        print("ERROR: cannot combine single inputs with '--all'")
        return 1
    if not find_all and not has_files:
        print("ERROR: no input files; did you want '--all'?")
        return 1
    if find_all:
        workspace_dir, found = _find_buildifier_sources()
        if len(found) == 0:
            print("ERROR: '--all' could not find anything")
            return 1
        print(f"This will reformat {len(found)} files within {workspace_dir}")
        if input("Are you sure [y/N]? ") not in ["y", "Y"]:
            print("... canceled")
            sys.exit(1)
        argv.extend(found)

    # Make cwd be what the user expected, not the runfiles tree.
    if "BUILD_WORKING_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])

    # Provide helpful diagnostics when in check mode.  Buildifier's -mode=check
    # uses exitcode 0 even when lint exists; we use whether or not its output
    # was empty to tell whether there was lint.
    if "-mode=check" in argv or "--mode=check" in argv:
        if _passes_check_mode(tool_cmds + argv):
            return 0
        switches = [x for x in argv if x.startswith("-")]
        files = [x for x in argv if not x.startswith("-")]
        print("ERROR: buildifier: the required formatting is incorrect")
        for one_file in files:
            if not _passes_check_mode(tool_cmds + switches + [one_file]):
                print(
                    f"ERROR: {one_file}:1: error: "
                    "the required formatting is incorrect"
                )
                print(
                    f"ERROR: {one_file}:1: note: fix via "
                    f"bazel-bin/tools/lint/buildifier {one_file}"
                )
                print(
                    f"ERROR: {one_file}:1: note: if that program does not "
                    "exist, you might need to compile it first: "
                    "bazel build //tools/lint/..."
                )
        print("NOTE: see https://drake.mit.edu/bazel.html#buildifier")
        return 1

    # In fix or diff mode, just let buildifier do its thing.
    return subprocess.call(tool_cmds + argv)


if __name__ == "__main__":
    sys.exit(main())
