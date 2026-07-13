"""Command-line tool to generate Drake's Python Interface files (.pyi)."""

import importlib
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

from mypy import stubgen


def _pydrake_modules():
    """Returns the list[str] of all pydrake module names."""
    # This import should (as a side effect) load all pydrake modules. We have
    # it as a function-local import because we don't want `_wrapper_main` to
    # pay the expense of importing this.
    importlib.__import__("pydrake._all_everything")

    # Scrape sys.modules for pydrake modules, but excluding private modules.
    result = [
        "pydrake",
    ]
    for name in sys.modules:
        if not name.startswith("pydrake."):
            continue
        if "._" in name:
            continue
        result.append(name)

    # Return a deterministic order to our caller.
    return sorted(result)


def _pyi_generated(directory: Path):
    """Returns a list[Path] of all *.pyi files in the given directory and its
    subdirectories. The directory should only contains *.pyi files; anything
    else is an error.
    """
    result = []
    for dirpath, dirnames, filenames in os.walk(directory):
        for one_filename in filenames:
            assert Path(one_filename).suffix == ".pyi"
            full = Path(dirpath) / one_filename
            result.append(full.relative_to(directory))

    # Return a deterministic order to our caller.
    return sorted(result)


def _copy_pyi(pyi_generated, output_root, pyi_outputs):
    """Copies pyi_generated to pyi_outputs, with cross-checking."""
    # Check for too few *.pyi files.
    missing_pyi = [x for x in pyi_outputs if x not in pyi_generated]
    if missing_pyi:
        raise RuntimeError(
            "The PYI_FILES = ... in the BUILD.bazel file specified that the "
            f"{missing_pyi} should have been created, but they were not. "
            "Possibly PYI_FILES should not have listed those items, or there "
            "are missing imports in all.py or _all_everything.py."
        )

    # Check for too many *.pyi files.
    extra_pyi = [x for x in pyi_generated if x not in pyi_outputs]
    if extra_pyi:
        raise RuntimeError(
            "The PYI_FILES = ... in the BUILD.bazel file did not specify "
            f"that {extra_pyi} would be created, but they were. "
            "Possibly PYI_FILES should list those items."
        )

    # Just right. The lists are identical.
    for pyi in pyi_outputs:
        shutil.copyfile(pyi, output_root / pyi)


def _actual_main():
    # Our arguments are the list of *.pyi outputs wanted from stubgen.bzl
    # (which currently come from the `PYI_FILES = ...` defined in BUILD.bazel).
    # They are probably relative paths, which we'll convert to an absolute
    # output_root path for where "pydrake/foo.pyi" should appear, and then a
    # list of relative paths under that. We must use rsplit because "pydrake"
    # might appear multiple times and we want to match the final one.
    output_root_str = sys.argv[1].rsplit("/pydrake/", maxsplit=1)[0]
    output_root = Path(output_root_str).absolute()
    pyi_outputs = sorted(
        [
            Path(path).absolute().relative_to(output_root)
            for path in sys.argv[1:]
        ]
    )

    # Find all native modules in pydrake (i.e., excluding pure python modules).
    native_modules = _pydrake_modules()
    for name in native_modules[:]:
        source = getattr(sys.modules[name], "__file__", "")
        if source.endswith(".py"):
            native_modules.remove(name)

    # Run stubgen. It writes junk in the current directory, so we need to run
    # it from a safe place.
    with tempfile.TemporaryDirectory(prefix="drake_stubgen_") as temp:
        os.chdir(temp)
        args = ["--output=."] + [f"--module={name}" for name in native_modules]
        returncode = stubgen.main(args=args) or 0
        assert returncode == 0, returncode

        # The generation was successful. Copy the *.pyi files to output.
        pyi_generated = _pyi_generated(Path(temp))
        _copy_pyi(pyi_generated, output_root, pyi_outputs)


def _wrapper_main():
    # By default, any console output from build actions is passed along to the
    # user (in contrast with tests, whose output is muted when they succeed).
    # However, it's difficult to teach mypy how to be quiet when successful, so
    # we'll use a subprocess to buffer the output until we know whether or not
    # there's been an error.
    hint = "--inner_process"
    if hint not in sys.argv:
        # Call ourself again with a new argument as a hint.
        env = dict(os.environ)
        env["PYTHONPATH"] = ":".join(sys.path)
        completed = subprocess.run(
            [sys.executable, "-B"] + sys.argv + [hint],
            encoding="utf-8",
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
        )
        if completed.returncode != 0:
            sys.stderr.write(completed.stdout)
        return completed.returncode
    else:
        # We're the inner process. Ditch the argument hint and proceed.
        sys.argv.remove(hint)
        return _actual_main()


if __name__ == "__main__":
    # XXX
    sys.exit(_actual_main())
