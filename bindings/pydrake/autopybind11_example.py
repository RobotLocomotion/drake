import os
import re
import subprocess
import sys
from subprocess import Popen, PIPE, STDOUT

from drake.tools.lint.find_data import find_data
from drake.tools.lint.util import find_all_sources


# These match data=[] in our BUILD.bazel file.
_AUTOPYBIND = "external/autopybind11"

def _make_autopybind_command():
    """Returns a list starting with the buildifier executable, followed by any
    required default arguments."""
    return [sys.executable, "-m",
        "external.autopybind11.autopybind11"]


def _help(command, my_env):
    """Perform the --help operation (display output) and return an exitcode."""
    process = Popen(command, stdout=PIPE, stderr=STDOUT,
                    env=my_env)
    stdout, _ = process.communicate()
    lines = stdout.splitlines()
    # Edit the first line to allow "--all" as a disjunction from "files...",
    # and make one or the other required.
    # head = re.sub(r'\[(files\.\.\.)\]', r'<\1 | --all>', lines.pop(0))
    for line in lines:
        print(line)
    return process.returncode

def main(workspace_name="drake"):
    # Slice out our overlay command-line argument "--all".
    argv = sys.argv[1:]
    find_all = False
    if "--all" in argv:
        find_all = True
        argv.remove("--all")

    # Find the wrapped tool.
    tool_cmds = _make_autopybind_command()
    my_env = os.environ.copy()
    my_env["PYTHONPATH"] = find_data(_AUTOPYBIND) + os.pathsep + my_env["PYTHONPATH"]
    my_env["PYTHONPATH"] = find_data("drake") + os.pathsep + my_env["PYTHONPATH"]
    my_env["PYTHONPATH"] = find_data("toposort") + os.pathsep + my_env["PYTHONPATH"]
    my_env["PYTHONPATH"] = find_data("ConfigArgParse") + os.pathsep + my_env["PYTHONPATH"]

    # Process --help.
    if "--help" in argv or "-help" in argv:
        return _help(tool_cmds + argv, my_env)
    else:
        # Set some defaults:
        # Stage should always be 2
        argv.append("-s")
        argv.append("2")
        # Sensible defaults
        argv.append("--module_name")
        argv.append("drake")
        out_dir = os.path.join(os.getcwd(), "autopybind11-out")
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        argv.append("-o")

        #Find CastXML as a repository
        argv.append(out_dir)
        argv.append("--castxml")
        argv.append(find_data("external/castxml/castxml_bin"))

        process = Popen(tool_cmds + argv, stdout=PIPE, stderr=STDOUT,
                        env=my_env)
        stdout, _ = process.communicate()
        lines = stdout.splitlines()
            # Edit the first line to allow "--all" as a disjunction from "files...",
            # and make one or the other required.
            # head = re.sub(r'\[(files\.\.\.)\]', r'<\1 | --all>', lines.pop(0))
        for line in lines:
            print(line)
if __name__ == "__main__":
    sys.exit(main())
