"""
Drake setup program to install necessary prerequisites.
"""

# Note for maintainers: We are in the process of migrating our bash
# setup code into this file.

import argparse
import enum
import functools
import hashlib
import json
import logging
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import textwrap
import time
import urllib.parse
import urllib.request

_MY_DIR: Path = Path(__file__).parent
"""The directory containing this script, used to locate our data assets."""

_allow_update = True
"""Whether apt-get update (or eventually, brew update) is allowed to be
called."""


class Flavor(enum.Enum):
    """Drake users (and developers) can choose a particular flavor of
    prerequisites to install, depending on which operations they need to
    accomplish."""

    # These are listed in order from narrowest to widest prerequisites.
    # Each flavor adds more prerequisites atop the prior flavor(s).
    BINARY = "to run a precompiled binary release"
    BUILD = "to build and install using CMake and GCC"
    DEVELOPER = "to develop Drake with Bazel"

    def __init__(self, description):
        self.description = description

    def __ge__(self, other):
        everything = list(Flavor)
        lhs = everything.index(self)
        rhs = everything.index(other)
        return lhs >= rhs


def _warn(message: str) -> None:
    """Logs a warning. For convenience, lines are unwrapped."""
    message = textwrap.dedent(message).replace("\n", " ").strip()
    logging.warning(message)


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
def _os_distribution() -> str | None:
    """Returns the distribution ID, e.g., "ubuntu"."""
    if platform.system() == "Linux":
        return platform.freedesktop_os_release()["ID"]
    raise NotImplementedError(platform.system())


@functools.cache
def _os_codename() -> str:
    """Returns the VERSION_CODENAME, e.g., "resolute"."""
    if platform.system() == "Linux":
        return platform.freedesktop_os_release()["VERSION_CODENAME"]
    raise NotImplementedError(platform.system())


@functools.cache
def _is_ubuntu() -> bool:
    return platform.system() == "Linux" and _os_distribution() == "ubuntu"


@functools.cache
def _is_mac() -> bool:
    return sys.platform == "darwin"


def _check_sudo() -> None:
    """Checks that 'sudo' has sufficient credentials."""
    # If sudo is already usable, then we're done.
    process = _run(args=["sudo", "-n", "/bin/true"], check=False, quiet=True)
    if process.returncode == 0:
        return
    # If not, then we need to refresh the credentials. N.B. This doesn't work in
    # our CI environment, but the prior check should have passed in that case.
    subprocess.check_call(["sudo", "-v"])


def _maybe_warn_conda() -> None:
    """Warns if conda is on the $PATH."""
    if shutil.which("conda") is None:
        return
    _warn("""
        Conda was detected on your $PATH. Drake is not tested regularly with
        Anaconda, so you may experience compatibility hiccups; when asking for
        help, be sure to mention that Conda is involved.
    """)


def _run(
    *,
    args: list,
    cwd: Path | None = None,
    check: bool = True,
    superuser: bool = False,
    flaky: bool = False,
    quiet: bool = False,
    interactive: bool = False,
) -> subprocess.CompletedProcess:
    """Runs a subprocess command given by `args`. When `check` is true, failure
    of the command is an `_error`. When `superuser` is true, the command will
    be run under 'sudo' unless the euid is already root. When `flaky` is true,
    the command will be retried a couple times when it fails. When `quiet` is
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
    num_attempts = 3 if flaky else 1
    for i in range(num_attempts):
        if i > 0:
            logging.info("... failed; waiting 30 sec before trying again ...")
            time.sleep(30)
        process = subprocess.run(
            args,
            cwd=cwd,
            stdin=subprocess.DEVNULL if not interactive else None,
            stdout=subprocess.PIPE if not interactive else None,
            stderr=subprocess.STDOUT if not interactive else None,
            text=True,
            check=False,
        )
        problem = check and (process.returncode != 0)
        if not problem:
            break
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


def _get_dpkg_versions(package_names: list[str]) -> dict[str, str | None]:
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
            r"--showformat=${Package} ${db:Status-Abbrev} ${Version}\n",
            *package_names,
        ],
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


@functools.cache
def _apt_update() -> None:
    """Runs 'apt-get update' to refresh available packages."""
    if not _allow_update:
        return
    process = _run(
        args=["apt-get", "update"],
        superuser=True,
        flaky=True,
    )
    if process.returncode == 0:
        return
    _error("""
        Drake is unable to run 'sudo apt-get update', probably because this
        computer contains incorrect entries in its sources.list files, or
        possibly because an internet service is down. Run 'sudo apt-get update'
        and try to resolve whatever problems it reports. Do not try to set up
        Drake until that command succeeds. THIS IS NOT A BUG IN DRAKE. Do not
        contact the Drake team for help.
    """)


def _apt_install(*, package_names: list[str], yes: bool) -> None:
    """Installs the given packages using 'apt-get'.
    The `yes` flag is passed along to apt as `--yes`."""
    assert package_names
    _apt_update()
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
    _run(args=args, superuser=True, interactive=True)


def _download_deb(*, temp_dir: Path, package: dict) -> Path:
    """Downloads a `*.deb` package a denoted by the given `package` entry loaded
    from the setup/ubuntu/packages.json file. Returns its path inside temp_dir.
    """
    name = package["name"]
    version = package["version"]
    urls = package["urls"]
    sha256 = package["sha256"]

    logging.info(f"Downloading {name} {version} ...")

    # Try each url in turn.
    result = None
    result_url = None
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
            result = temp_filename
            result_url = url
            break
        errors.append(
            f"Candidate {url} failed:\n"
            f"Checksum mismatch; was {download_sha256} but wanted {sha256}."
        )

    # Always show error messages, even if we succeeded. Any error indicates a
    # real problem with either our json file or a server.
    if errors:
        # TODO(#24850) Add unit test coverage here once we have multiple URLs.
        for message in errors:
            _warn(message)
        if result_url is not None:
            logging.info(f"Candidate {result_url} succeeded")
    if result is None:
        _error(f"All {name} downloads failed")

    return result


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
        if deb_arch not in package["arches"]:
            continue
        if _os_codename() not in package["codenames"]:
            continue
        name = package["name"]
        assert name not in packages, name
        packages[name] = package
    if not packages:
        return

    # Check what's already installed in case we can skip some.
    all_names = list(packages.keys())
    for name, installed_version in _get_dpkg_versions(all_names).items():
        desired_version = packages[name]["version"]
        if installed_version is None:
            # The package is missing; we will need to install it.
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
                str(_download_deb(temp_dir=Path(temp), package=package))
                for package in packages.values()
            ]
            _apt_install(package_names=paths, yes=yes)


def _maybe_fix_gcc(*, yes: bool) -> None:
    """Corrects a common GCC installation mistake. On Noble, Drake doesn't
    install anything related to GCC 14, but if the user has chosen to install
    some GCC 14 libraries but has failed to install all of them correctly as a
    group, Drake's documentation header file parser (run during linting) will
    fail with a libclang-related complaint. Therefore, we'll help the developer
    clean up their mess, to avoid apparent Drake linter errors.
    """
    if _os_codename() != "noble":
        return
    packages = ["libgcc-14-dev", "libstdc++-14-dev", "libgfortran-14-dev"]
    missing = [
        package_name
        for package_name, version in _get_dpkg_versions(packages).items()
        if version is None
    ]
    if len(missing) == len(packages):
        # No action required. Nothing from the group is installed yet.
        return
    if len(missing) == 0:
        # No action required. The whole group is already installed.
        return
    _apt_install(package_names=missing, yes=yes)


def _generate_locales():
    """Ensures that we have available a locale that supports UTF-8 for
    generating a C++ header containing Python API documentation during
    the build."""
    # The canonical name is used when setting a locale, in error messages, etc.
    canonical = "en_US.UTF-8"
    # The alias name is what is reported by the `locale -a` directory lookup.
    alias = "en_US.utf8"
    for line in _run(args=["locale", "-a"], quiet=True).stdout.splitlines():
        if line.strip() == alias:
            logging.debug(f"The {canonical} locale already exists.")
            return
    _run(
        args=["locale-gen", canonical],
        superuser=True,
    )


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
        logging.info(f"Writing {path!s} ...")
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


def _apt_install_flavor(*, flavor: Flavor, yes: bool) -> None:
    """Installs the apt packages from Ubuntu required for the given `flavor`
    and all of its antecedent flavors. The `yes` flag is passed along to apt
    as `--yes`."""
    assert isinstance(flavor, Flavor)

    # Collect the packages needed by `flavor` and its antecedents.
    packages = []
    for item in Flavor:
        txt_file = (
            _MY_DIR
            / _os_distribution()
            / f"packages-{_os_codename()}-{item.name.lower()}.txt"
        )
        if not txt_file.exists():
            description = platform.freedesktop_os_release().get(
                "PRETTY_NAME", "<unknown>"
            )
            _warn(f"No such file {txt_file!s}.")
            _error(f"This script does not support {description}.")
        logging.debug(f"Reading {txt_file}.")
        for line in txt_file.read_text(encoding="utf-8").splitlines():
            line = line.split("#")[0].strip()
            if not line:
                continue
            packages.append(line)
        if item == flavor:
            break
    packages = sorted(packages)

    # Check what's not already installed and install it.
    installed = _get_dpkg_versions(packages)
    missing_packages = [
        name for name, version in installed.items() if version is None
    ]
    if missing_packages:
        _apt_install(package_names=missing_packages, yes=yes)
    else:
        logging.debug("All selected packages-*.txt are already installed.")


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
        action="store_const",
        dest="flavor",
        default=Flavor.BUILD,
        const=Flavor.DEVELOPER,
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
        dest="allow_update",
        action="store_false",
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

    # Under --user-environment-only, the flavor is irrelevant. We'll set up the
    # environment and then we're done.
    if args.user_environment_only:
        _setup_user_environment()
        return

    # Validate the flavor to install.
    if args.flavor >= Flavor.DEVELOPER and not (_is_mac() or _is_ubuntu()):
        raise NotImplementedError(platform.system())

    # Prepare for installation.
    global _allow_update
    _allow_update = args.allow_update
    _maybe_warn_conda()

    # Install the prerequisites.
    if not _is_mac():
        _apt_install_flavor(
            flavor=args.flavor,
            yes=args.yes,
        )
    if args.flavor >= Flavor.DEVELOPER and _is_ubuntu():
        _install_downloaded_debs(yes=args.yes)
        _maybe_fix_gcc(yes=args.yes)

    # Configure the prerequisites.
    if args.flavor >= Flavor.DEVELOPER:
        if _is_ubuntu():
            _generate_locales()
        _setup_user_environment()
        _prefetch_bazel()

    # Finished.
    logging.info(
        f"Successfully installed {args.flavor.name.lower()}"
        f" prereqs ({args.flavor.description})."
    )


if __name__ == "__main__":
    main()
