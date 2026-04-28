"""
Drake setup program to install necessary prerequisites.

We are in the process of migrating our bash setup code into this file.
At the moment, this file is not usable as a stand-alone script.
Always run `drake/setup/install_prereqs`, instead.
"""

import argparse
import functools
import logging
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import textwrap

_MY_DIR: Path = Path(__file__).parent
"""The directory containing this script, used to locate our data assets."""


def _error(message: str) -> None:
    """Logs an error and exits. For convenience, lines are unwrapped."""
    message = textwrap.dedent(message).replace("\n", " ").strip()
    logging.error(message)
    sys.exit(1)


@functools.cache
def _workspace_dir() -> Path:
    """Returns the Path of the Drake workspace (source builds only; does not
    work for binary installs).
    """
    assert _MY_DIR.name == "setup", _MY_DIR
    return _MY_DIR.parent


def _run(
    *,
    args: list,
    cwd: Path | None = None,
    quiet: bool = False,
) -> None:
    """Runs a subprocess command given by `args`. Failure of the command is an
    `_error`. When `quiet` is true, the command line will not be printed by
    default.
    """
    command = args[0]
    logging.log(
        msg=f"Running: {shlex.join(args)} ...",
        level=logging.DEBUG if quiet else logging.INFO,
    )
    process = subprocess.run(
        args,
        cwd=cwd,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    problem = process.returncode != 0
    for line in process.stdout.splitlines():
        logging.log(
            msg=f"... from {command}: {line}",
            level=logging.INFO if problem else logging.DEBUG,
        )
    logging.debug(f"... finished {command}.")
    if problem:
        _error(f"{command} failed with returncode {process.returncode}")


def _setup_user_environment():
    """Update user-specific config snippets needed only by Drake Developers."""
    # Compute the bazel python version snippet.
    # Our MODULE.bazel uses this file to determine the default python version.
    python_version_content = "{}.{}\n".format(
        sys.version_info.major,
        sys.version_info.minor,
    )

    # Compute the bazel rcfile snippet. This is always created, but only needs
    # content for Drake Developers on Linux.
    bazelrc_content = ""
    if sys.platform == "linux":
        os_release = platform.freedesktop_os_release()
        developer_txt = (
            _MY_DIR
            / "ubuntu"
            / f"packages-{os_release['VERSION_CODENAME']}-developer.txt"
        )
        clang_re = re.compile("^clang-([0-9]+)$")
        for line in developer_txt.read_text(encoding="utf-8").splitlines():
            clang_match = clang_re.match(line)
            if clang_match is not None:
                (clang_major,) = clang_match.groups()
                break
        else:
            _error("Could not find clang version in developer.txt")
        bazelrc_content = textwrap.dedent(f"""\
        common:clang --repo_env=CC=clang-{clang_major}
        build:clang --action_env=CC=clang-{clang_major}
        build:clang --host_action_env=CC=clang-{clang_major}
        """)

    # Write out the two snippets.
    workspace_dir = _workspace_dir()
    (workspace_dir / "gen").mkdir(exist_ok=True)
    python_version = workspace_dir / "gen/python_version.txt"
    bazelrc = workspace_dir / "gen/environment.bazelrc"
    for path, content in (
        (python_version, python_version_content),
        (bazelrc, bazelrc_content),
    ):
        logging.info(f"Writing {str(path)} ...")
        logging.debug(f" content={content!r}")
        path.write_text(content, encoding="utf-8")


def _prefetch_bazel():
    """Prefetch the bazelisk download of bazel. (This is especially helpful for
    the "Provisioned" images in CI.)
    """
    if os.geteuid() == 0:
        logging.warning("Not pre-fetching bazel for root user.")
        return
    logging.info("Pre-fetching bazel ...")
    _run(args=["bazel", "version"], cwd=_workspace_dir(), quiet=True)


def main():
    # Log at INFO, not just WARNING.
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    # Initialize argparse.
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--developer",
        action="store_true",
        help="Install prerequisites needed only by Drake Developers.",
    )
    parser.add_argument(
        "--user-environment-only",
        action="store_true",
        help=(
            "Update per-user config snippets needed only by Drake Developers, "
            "but don't install any system-wide packages."
        ),
    )
    for name in ("--without-update", "-y"):
        parser.add_argument(
            name,
            action="store_true",
            help="Ignored for forwards compatibility.",
        )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbosity.",
    )
    args = parser.parse_args()
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    # We are in the process of migrating our bash setup code into this file.
    # Anything not set up here was already setup by install_prereqs.sh.
    if args.developer or args.user_environment_only:
        _setup_user_environment()
    if args.developer:
        _prefetch_bazel()


if __name__ == "__main__":
    main()
