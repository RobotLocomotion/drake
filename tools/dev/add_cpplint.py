#!/usr/bin/env python3

import os
import subprocess

"""
Iterate through scripts that are not covered by cpplint, and ensure that they
are properly covered.
"""


def subshell(cmd, shell=True, strip=True):
    output = subprocess.check_output(cmd, shell=shell).decode("utf8")
    if strip:
        output = output.strip()
    return output


class FileProcessor:
    def __init__(self):
        # Files processed
        self.build_files = set()

    def ensure_source_covered(self, source_file):
        print("Ensure source covered: {}".format(source_file))
        build_file = (subshell("bazel query --output location {}"
                               .format(source_file))
                      .split(':')[0])
        self.ensure_cpplint(build_file)

    def ensure_cpplint(self, build_file):
        print("  Check build file: {}".format(build_file))
        if build_file in self.build_files:
            print("   Already covered")
            return
        with open(build_file) as f:
            text = f.read()
        # Not robust, but hoping it's mentioned
        if "add_lint_tests()" not in text:
            text = text.rstrip() + "\n\nadd_lint_tests()\n"
            with open(build_file, 'w') as f:
                f.write(text)
            print("    Updated file")
        else:
            print("    add_lint_tests() already present")
        self.build_files.add(build_file)


def do_main():
    workspace = subshell("bazel info workspace").strip()
    os.chdir(workspace)

    subshell("./tools/dev/check_missing_sources.sh")
    files_missed = (
        subshell(r'diff -u /tmp/cc_files.txt /tmp/cpplint_files.txt'
                 + r' | grep "^-\w" | sed "s#^\-##g"')
        .strip().split('\n'))

    file_proc = FileProcessor()
    for source_file in files_missed:
        file_proc.ensure_source_covered(source_file)


if __name__ == "__main__":
    do_main()
