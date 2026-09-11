#!/usr/bin/env python3

"""
upgrade.py - Upgrades Drake's version of bazelisk.

This program is only tested / supported on Ubuntu.
"""

# noqa: shebang
# We suppress shebang lint checking because we have a magic trampoline atop our
# main function that allows us to re-execute ourselves using Bazel.

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import urllib.request


def _get_url_sha256(url: str) -> str:
    hasher = hashlib.sha256()
    with urllib.request.urlopen(url) as response:
        while chunk := response.read(4096):
            hasher.update(chunk)
    return hasher.hexdigest()


def main():
    bazelisk_license_path = os.environ.get("DRAKE_BAZELISK_LICENSE_PATH")
    if bazelisk_license_path is None:
        # Operate relative to the root of the Drake source tree.
        os.chdir(Path(__file__).resolve().parents[3])
        os.execvp(
            "bazel",
            ["bazel", "run", "//tools/workspace/bazelisk_internal:upgrade"],
        )

    # This import only works when run via Bazel, so must come after the re-exec.
    from python import runfiles

    manifest = runfiles.Create()

    mydir = (
        Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
        / "tools/workspace/bazelisk_internal"
    )

    # Find out which version we are pinned to (new_release has already upgraded
    # our repository.bzl).
    new_version = None
    my_version_re = re.compile(r'commit\s*=\s*["\']([^"\']+)["\']')
    repo_bzl_lines = (
        (mydir / "repository.bzl").read_text(encoding="utf-8").splitlines()
    )
    for line in repo_bzl_lines:
        m = my_version_re.search(line)
        if m:
            new_version = m.group(1)
            break
    assert new_version

    # Upgrade setup/ubuntu/packages.json.
    setup_dir = (
        Path(os.environ["BUILD_WORKSPACE_DIRECTORY"]) / "setup" / "ubuntu"
    )
    packages = json.loads(
        (setup_dir / "packages.json").read_text(encoding="utf-8")
    )
    for package in packages:
        if package["name"] != "bazelisk":
            continue
        for i, url in enumerate(package["urls"]):
            package["urls"][i] = re.sub(r"v\d+\.\d+\.\d+", new_version, url)
        package["sha256"] = _get_url_sha256(package["urls"][0])
    (setup_dir / "packages.json").write_text(json.dumps(packages, indent=4))

    # Upgrade our third_party copy.
    bazelisk_py_path = os.environ.get("DRAKE_BAZELISK_PY_PATH")
    bazelisk_files = {
        manifest.Rlocation(bazelisk_license_path),
        manifest.Rlocation(bazelisk_py_path),
    }
    third_party_dir = (
        Path(os.environ["BUILD_WORKSPACE_DIRECTORY"])
        / "third_party"
        / "com_github_bazelbuild_bazelisk"
    )
    for file in bazelisk_files:
        shutil.copy2(file, third_party_dir)


if __name__ == "__main__":
    main()
