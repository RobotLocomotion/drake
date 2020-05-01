"""Run Google's buildifier tool to fix, check, or fix-diff BUILD files.

All arguments except "--all" are passed through to Google's tool.  As with the
Google tool, the default mode is "-mode=fix".  In "-mode=check", we promote
lint errors to a non-zero exitcode.
"""


import os
import re
import subprocess
import sys
from subprocess import Popen, PIPE, STDOUT

from drake.tools.lint.find_data import find_data
from drake.tools.lint.util import find_all_sources

# These match data=[] in our BUILD.bazel file.
_BUILDIFIER = "external/buildifier/buildifier"
_TABLES = "tools/lint/buildifier-tables.json"


def _make_buildifier_command():
    """Returns a list starting with the buildifier executable, followed by any
    required default arguments."""
    return [
        find_data(_BUILDIFIER),
        "-add_tables={}".format(find_data(_TABLES))]


def _help(command):
    """Perform the --help operation (display output) and return an exitcode."""
    process = Popen(command, stdout=PIPE, stderr=STDOUT)
    stdout, _ = process.communicate()
    lines = stdout.splitlines()
    # Edit the first line to allow "--all" as a disjunction from "files...",
    # and make one or the other required.
    head = re.sub(r'\[(files\.\.\.)\]', r'<\1 | --all>', lines.pop(0))
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


def _find_buildifier_sources(workspace_name):
    """Return a list of all filenames to be covered by buildifier."""
    workspace, sources_relpath = find_all_sources(workspace_name)
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
        return (len(output) == 0)
    except subprocess.CalledProcessError as e:
        # https://github.com/bazelbuild/buildtools/blob/1a7c0ec10697afcb87af8a09f12c3f9b9ca56fb2/buildifier/buildifier.go#L227
        REFORMAT_IS_NEEDED = 4
        if e.returncode == REFORMAT_IS_NEEDED:
            return False
        raise e


def main(workspace_name="drake"):
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
        workspace_dir, found = _find_buildifier_sources(workspace_name)
        if len(found) == 0:
            print("ERROR: '--all' could not find anything")
            return 1
        print(f"This will reformat {len(found)} files "
              f"within {workspace_dir}")
        if input("Are you sure [y/N]? ") not in ["y", "Y"]:
            print("... canceled")
            sys.exit(1)
        argv.extend(found)

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
                print("ERROR: %s:1: error: %s" % (
                    one_file, "the required formatting is incorrect"))
                print("ERROR: %s:1: note: fix via %s %s" % (
                    one_file, "bazel-bin/tools/lint/buildifier", one_file))
                print(("ERROR: %s:1: note: if that program does not exist, "
                       "you might need to compile it first: "
                       "bazel build //tools/lint/...") %
                      one_file)
        print("NOTE: see https://drake.mit.edu/bazel.html#buildifier")
        return 1

    # In fix mode, disallow running from within the Bazel sandbox.
    if "-mode=diff" not in argv and "--mode=diff" not in argv:
        if os.getcwd().endswith(".runfiles/drake"):
            print("ERROR: do not use 'bazel run' for buildifier in fix mode")
            print("ERROR: use bazel-bin/tools/lint/buildifier instead")
            return 1

    # In fix or diff mode, just let buildifier do its thing.
    return subprocess.call(tool_cmds + argv)


if __name__ == "__main__":
    sys.exit(main())
