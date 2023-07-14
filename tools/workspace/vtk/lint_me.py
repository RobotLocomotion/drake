#!/usr/bin/env python3

import subprocess
from pathlib import Path


def main():
    # TODO(svenevs): this is how I was managing this file... we discussed
    # adding some kind of tests.  My strong vote is for mypy, which when hooked
    # up easily finds the errors I discussed in the meeting.
    #
    # MYPYPATH="$PWD:$PWD/image" ./lint_me.py
    #
    # The drake/__init__.py seems to confuse things.
    this_file_dir = Path(__file__).parent.absolute()
    py_files = [str(p) for p in this_file_dir.rglob("*.py")]
    subprocess.check_call(
        [
            "black",
            "--line-length",
            "80",
            *py_files,
        ]
    )

    subprocess.check_call(
        [
            "mypy",
            "--config-file",
            str(this_file_dir / "mypy.ini"),
            "--explicit-package-bases",
            str(this_file_dir),
        ]
    )

    subprocess.check_call(["flake8", *py_files])


if __name__ == "__main__":
    main()
