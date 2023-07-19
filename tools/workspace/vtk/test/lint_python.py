#!/usr/bin/env python3

import os
import subprocess
from pathlib import Path

from bazel_tools.tools.python.runfiles import runfiles


def _rlocation(relative_path: str) -> Path:
    """Return the real path to ``drake/{relative_path}``."""
    manifest = runfiles.Create()
    resource_path = f"drake/{relative_path}"
    resolved_path = manifest.Rlocation(resource_path)
    assert resolved_path, f"Missing {resource_path}"
    return Path(resolved_path).resolve()


def main():
    # TODO(svenevs): I'm running this incorrectly as a hack, create a virtual
    # environment at drake's root titled `venv` and install:
    #
    # pip install mypy black flake8 flake8-docstrings
    #
    # This test hard-codes these paths since none of these are drake
    # dependencies and this is likely not what we're going with long term.
    this_file = _rlocation("tools/workspace/vtk/test/lint_python.py")
    drake_root = this_file.parent.parent.parent.parent.parent
    black_exe = drake_root / "venv" / "bin" / "black"
    mypy_exe = drake_root / "venv" / "bin" / "mypy"
    flake8_exe = drake_root / "venv" / "bin" / "flake8"
    tools_workspace_vtk = drake_root / "tools" / "workspace" / "vtk"
    py_files = [str(p) for p in tools_workspace_vtk.rglob("*.py")]

    # TODO(svenevs): running with --check in CI is what we would want to do.
    # Locally, we want it to just do the formatting?
    subprocess.check_call(
        [
            str(black_exe),
            "--line-length",
            "80",
            *py_files,
        ]
    )

    # The drake/__init__.py seems to confuse things.
    mypy_env = os.environ.copy()
    mypy_path = f"{tools_workspace_vtk}:{tools_workspace_vtk / 'image'}"
    mypy_env["MYPYPATH"] = mypy_path
    subprocess.check_call(
        [
            str(mypy_exe),
            "--config-file",
            str(tools_workspace_vtk / "test" / "mypy.ini"),
            "--explicit-package-bases",
            str(tools_workspace_vtk),
        ],
        env=mypy_env,
    )

    subprocess.check_call([str(flake8_exe), *py_files])


if __name__ == "__main__":
    main()
