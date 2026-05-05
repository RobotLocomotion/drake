"""
Drake setup program to install necessary prerequisites.

We are in the process of migrating our bash setup code into this file.
At the moment, this file is not usable as a stand-alone script.
Always run `drake/setup/install_prereqs`, instead.
"""

import argparse
import functools
import hashlib
import json
import logging
import os
from pathlib import Path
import platform
import re
import shlex
import subprocess
import sys
import tempfile
import textwrap
import urllib.parse
import urllib.request

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


@functools.cache
def _is_ubuntu() -> bool:
    if platform.system() == "Linux":
        os_release = platform.freedesktop_os_release()["ID"]
        return os_release == "ubuntu"
    return False


@functools.cache
def _os_codename() -> str:
    if platform.system() == "Linux":
        return platform.freedesktop_os_release()["VERSION_CODENAME"]
    raise NotImplementedError(platform.system())


def _check_sudo() -> None:
    """Checks that 'sudo' has sufficient credentials."""
    subprocess.check_call(["sudo", "-v"])


def _run(
    *,
    args: list,
    cwd: Path | None = None,
    check: bool = True,
    superuser: bool = False,
    quiet: bool = False,
    interactive: bool = False,
) -> subprocess.CompletedProcess:
    """Runs a subprocess command given by `args`. When `check` is true, failure
    of the command is an `_error`. When `superuser` is true, the command will
    be run under 'sudo' unless the euid is already root. When `quiet` is
    true, the command line will not be printed by default. When `interactive`
    is true, input is allowed and output is unbuffered. Returns the completed
    process object."""
    command = args[0]
    if superuser and os.geteuid() != 0:
        _check_sudo()
        args = ["sudo"] + args
    logging.log(
        msg=f"Running: {shlex.join(args)} ...",
        level=logging.DEBUG if quiet else logging.INFO,
    )
    process = subprocess.run(
        args,
        cwd=cwd,
        stdin=subprocess.DEVNULL if not interactive else None,
        stdout=subprocess.PIPE if not interactive else None,
        stderr=subprocess.STDOUT if not interactive else None,
        text=True,
    )
    problem = check and (process.returncode != 0)
    if process.stdout is not None:
        for line in process.stdout.splitlines():
            logging.log(
                msg=f"... from {command}: {line}",
                level=logging.INFO if problem else logging.DEBUG,
            )
    logging.debug(f"... finished {command}.")
    if problem:
        _error(f"{command} failed with returncode {process.returncode}")
    return process


def _get_dpkg_versions(package_names: list[str]) -> dict[str, str]:
    """Returns the installed version of packages. The input is a list of
    package names, and the return value is a dict mapping all of those
    names to their installed version (or None, if not installed)."""
    assert package_names
    result = {}
    for name in package_names:
        result[name] = None
    process = _run(
        args=[
            "dpkg-query",
            "--show",
            "--showformat=${Package} ${db:Status-Abbrev} ${Version}\n",
        ]
        + package_names,
        check=False,
        quiet=True,
    )
    for line in process.stdout.splitlines():
        tokens = line.split()
        if len(tokens) != 3:
            continue
        name, status, version = tokens
        if status == "ii":
            result[name] = version
    logging.debug(f"dpkg_versions = {result!r}")
    return result


def _apt_install(*, package_names: list[str], yes: bool) -> None:
    """Installs the given packages using 'apt-get'.
    The `yes` flag is passed along to apt as `--yes`."""
    assert package_names
    args = [
        "apt-get",
        "install",
        "--no-install-recommends",
    ]
    if yes:
        args.append("--yes")
    args.extend(package_names)
    process = _run(args=args, superuser=True, check=yes)
    if process.returncode == 0:
        return
    # We can only reach here when yes=False (i.e., check=False). The apt-get
    # command didn't work, and the most likely reason is it needs Y/n input
    # from the user, so we'll try it again allowing for user input.
    _run(args=args, superuser=True, interactive=True, quiet=True)


def _download(*, temp_dir: Path, package: dict) -> Path:
    """Downloads a `*.deb` package a denoted by the given `package` entry loaded
    from the setupo/ubuntu/packages.json file. Returns its path inside temp_dir.
    """
    name = package["name"]
    version = package["version"]
    urls = package["urls"]
    sha256 = package["sha256"]

    logging.info(f"Downloading {name} {version} ...")

    # Try each url in turn.
    errors = []
    for url in urls:
        logging.debug(f"Trying {url} ...")
        basename = urllib.parse.urlparse(url).path.split("/")[-1]
        temp_filename = temp_dir / basename
        hasher = hashlib.sha256()
        with temp_filename.open("wb") as f:
            try:
                with urllib.request.urlopen(url=url, timeout=30) as response:
                    while True:
                        data = response.read(4096)
                        if not data:
                            break
                        hasher.update(data)
                        f.write(data)
            except OSError as e:
                errors.append(f"Candidate {url} failed:\n{e}")
                continue
        download_sha256 = hasher.hexdigest()
        if download_sha256 == sha256:
            return temp_filename
        errors.append(
            f"Candidate {url} failed:\n"
            f"Checksum mismatch; was {download_sha256} but wanted {sha256}."
        )

    # No downloads succeeded.
    messages = "\n\n".join(errors)
    _error(f"All downloads failed:\n\n{messages}")


def _install_downloaded_debs(*, yes: bool) -> None:
    """Downloads and installs required debs for --developer that are not
    available in Ubuntu's apt site.
    The `yes` flag is passed along to apt as `--yes`."""
    deb_arch = {
        "x86_64": "amd64",
        "aarch64": "arm64",
    }[platform.machine().lower()]

    # Load the list of packages and filter for the relevant ones.
    json_filename = _MY_DIR / "ubuntu/packages.json"
    packages = {}
    for package in json.loads(json_filename.read_text(encoding="utf-8")):
        assert package["type"] == "download_deb"
        name = package["name"]
        if deb_arch not in package["arches"]:
            continue
        if _os_codename() not in package["codenames"]:
            continue
        assert name not in packages, name
        packages[name] = package
    if not packages:
        return

    # Check what's already installed in case we can skip some.
    all_names = list(packages.keys())
    for name, installed_version in _get_dpkg_versions(all_names).items():
        desired_version = packages[name]["version"]
        if installed_version is None:
            continue

        # Check if already installed at the exact version.
        if installed_version == desired_version:
            logging.debug(f"{name} already installed at the desired version.")
            del packages[name]
            continue

        # Check if already installed at a newer version.
        comparison = _run(
            args=[
                "dpkg",
                "--compare-versions",
                installed_version,
                "gt",
                desired_version,
            ],
            quiet=True,
            check=False,
        )
        if comparison.returncode == 0:
            logging.info(
                f"Not downgrading {name} from {installed_version=} "
                f"to {desired_version=}."
            )
            del packages[name]
            continue

    # Download and install the necessary file(s).
    if packages:
        with tempfile.TemporaryDirectory(prefix="drake_prereqs_") as temp:
            paths = [
                str(_download(temp_dir=Path(temp), package=package))
                for package in packages.values()
            ]
            _apt_install(package_names=paths, yes=yes)


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
    if _is_ubuntu():
        developer_txt = (
            _MY_DIR / "ubuntu" / f"packages-{_os_codename()}-developer.txt"
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
    parser.add_argument(
        "--without-update",
        action="store_true",
        help="Ignored for forwards compatibility.",
    )
    parser.add_argument(
        "-y",
        action="store_true",
        dest="yes",
        help="Install without prompting for confirmation.",
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
    if _is_ubuntu() and args.developer:
        _install_downloaded_debs(yes=args.yes)
    if args.developer or args.user_environment_only:
        _setup_user_environment()
    if args.developer:
        _prefetch_bazel()


if __name__ == "__main__":
    main()
