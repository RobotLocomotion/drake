r"""Re-package a Drake .tar.gz archive into a Debian archive.

This script assumes that the Ubuntu distribution the .tar.gz archive was built
for is the same as what is running the script.  For example, a
drake-latest-noble.tar.gz must be re-packaged on a noble machine.

Command line arguments should specify absolute paths, e.g.,

    bazel run //tools/release_engineering:repack_deb -- \
        --tgz "$PWD/drake-0.0.YYYYMMDD-noble.tar.gz" \
        --deb "$PWD/drake-dev_0.0.YYYYMMDD-1_amd64-noble.deb"

will repackage the --tgz input file, writing to the --deb output file.
"""

import argparse
import csv
import email.utils
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tarfile
import tempfile

from python import runfiles


def _get_os_release():
    with open("/etc/os-release") as f:
        reader = csv.reader(f, delimiter="=")
        return dict(reader)


def _rlocation(relative_path):
    manifest = runfiles.Create()
    resource_path = f"drake/tools/release_engineering/{relative_path}"
    resolved_path = manifest.Rlocation(resource_path)
    assert resolved_path, f"Missing {resource_path}"
    return os.path.realpath(resolved_path)


def _run(args):
    """Implements all steps for repacking tgz => deb."""
    # Find our runfiles.
    deb_control_in = _rlocation("debian/control.in")
    deb_copyright = _rlocation("debian/copyright")
    deb_changelog_in = _rlocation("debian/changelog.in")

    # Discern the architecture we're running on.
    arch = platform.machine().lower()
    if arch in {"amd64", "x86_64"}:
        arch = "amd64"
    elif arch in {"arm64", "aarch64"}:
        arch = "arm64"
    else:
        raise RuntimeError(f"Unsupported architecture {arch!r}.")

    # Discern the version badging to use, get the dependencies for drake.
    codename = _get_os_release()["VERSION_CODENAME"]
    assert codename in str(args.tgz), (
        "Debian re-packaging must be performed on the same distribution, but "
        f"'{codename}' was not found in '{args.tgz}'."
    )
    with tarfile.open(args.tgz) as archive:
        version = archive.getmember("drake/share/doc/drake/VERSION.TXT")
        version_txt = archive.extractfile(version).read().decode("utf-8")
        version_mtime = version.mtime

        packages = archive.getmember(
            f"drake/share/drake/setup/packages-{codename}-binary.txt"
        )
        packages_txt = archive.extractfile(packages).read().decode("utf-8")

    version_tokens = version_txt.split()
    assert len(version_tokens) == 2, version_txt
    drake_version, git_sha = version_tokens

    # Compute the new control.  The `packages_txt` will have one package-name
    # per line; transform this to an indented and comma separated list.
    # NOTE: rstrip is required here so that the last line does *NOT* have a
    # comma (otherwise debian/rules will crash).
    depends = packages_txt.rstrip().replace("\n", ",\n         ")
    with open(deb_control_in, encoding="utf-8") as f:
        deb_control_contents = f.read().format(arch=arch, depends=depends)

    # Compute the new changelog.
    with open(deb_changelog_in, encoding="utf-8") as f:
        deb_changelog_contents = f.read().format(
            drake_version=drake_version,
            git_sha=git_sha,
            date=email.utils.formatdate(version_mtime),
        )

    # Use `alien` to prepare the debian packaging rules.
    cwd = f"{args.tempdir}/alien"
    os.mkdir(cwd)
    alien = [
        "fakeroot",
        "/usr/bin/alien",
        "--to-deb",
        "--single",
        f"--version={drake_version}",
        "--keep-version",
        # NOTE: the extracted permissions particularly for directories are not
        # appropriate, --fixperms results in debian/rules having `dh_fixperms`
        # which will repair the broken permissions.
        "--fixperms",
        "--verbose",
        args.tgz,
    ]
    env = dict(os.environ)
    env["EMAIL"] = "drake-users@mit.edu"
    subprocess.check_call(alien, cwd=cwd, env=env)
    # Alien does not produce consistent directory names when extracting the
    # .tar.gz archive.  The exact pattern is not known or described in the
    # documentation, it appears to depend on both the name of the archive and
    # the --version argument.  Some examples:
    #
    # - drake-latest-noble.tar.gz => drake-latest-noble-0.0.20220513083006
    # - drake-20220512-noble.tar.gz => drake-0.0.20220512082823
    # - foo.tar.gz => foo-0.0.20220513083006
    #
    # It appears to keep the original name _until_ it finds numbers e.g.
    # 20220512.  As such, we list the directories under our alien folder and
    # assert that its length is one.  Note that regardless of the name of the
    # folder produced, the final `.deb` file will have the correct name format
    # drake-dev_{version}-1_{arch}.deb.
    directories = [d for d in Path(cwd).iterdir() if d.is_dir()]
    assert len(directories) == 1, "Unable to discover alien output directory."
    package_dir = str(directories[0])

    # Install into /opt/drake, not /drake.
    os.mkdir(f"{package_dir}/opt")
    os.rename(f"{package_dir}/drake", f"{package_dir}/opt/drake")

    # Overwrite some metadata.
    with open(f"{package_dir}/debian/control", "w", encoding="utf-8") as f:
        f.write(deb_control_contents)
    shutil.copy(deb_copyright, f"{package_dir}/debian/")
    with open(f"{package_dir}/debian/changelog", "w", encoding="utf-8") as f:
        f.write(deb_changelog_contents)

    # Create the deb.
    subprocess.check_call(
        ["fakeroot", "debian/rules", "binary"], cwd=package_dir
    )
    shutil.move(f"{cwd}/drake-dev_{drake_version}-1_{arch}.deb", args.deb)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--tgz",
        type=str,
        required=True,
        help="the foo.tar.gz filename to be re-packaged",
    )
    parser.add_argument(
        "--deb",
        type=str,
        required=True,
        help="the foo.deb filename to create",
    )
    args = parser.parse_args()
    args.tgz = Path(args.tgz).resolve()
    args.deb = Path(args.deb).resolve()

    # Fail early if we cannot find the input / output locations rather than
    # failing on trying to open the file or move it at the end.
    assert args.tgz.exists(), args.tgz
    assert args.deb.parent.exists(), args.deb.parent

    with tempfile.TemporaryDirectory(prefix="drake-repack-") as tempdir:
        args.tempdir = tempdir
        _run(args)


if __name__ == "__main__":
    main()
