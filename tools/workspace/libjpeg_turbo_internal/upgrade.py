#!/usr/bin/env python3
"""
upgrade.py - Upgrades Drake's version of libjpeg-turbo.

This program is only tested / supported on Ubuntu.
"""

import logging
import os
from pathlib import Path
import re
import urllib.request


def _main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(levelname)s: %(message)s",
    )

    # Operate relative to the root of the Drake source tree.
    os.chdir(Path(__file__).resolve().parent.parent.parent.parent)
    mydir = Path("tools/workspace/libjpeg_turbo_internal")

    # Download the latest files from upstream.
    logging.info("Refreshing tensorflow files ...")
    remote = "https://github.com/tensorflow/tensorflow/raw/master/"
    local = Path("third_party/com_github_tensorflow_tensorflow")
    downloads = [
        "LICENSE",
        "third_party/jpeg/jpeg.BUILD",
        "third_party/jpeg/workspace.bzl",
    ]
    for item in downloads:
        with urllib.request.urlopen(f"{remote}{item}") as response:
            data = response.read()
        with open(local / item, "wb") as f:
            f.write(data)

    # Find out which version we are pinned to.
    old_version = None
    my_version_re = re.compile(r'("commit".*default = ")([^"]*)')
    with open(mydir / "repository.bzl", encoding="utf-8") as f:
        for line in f.readlines():
            m = my_version_re.search(line)
            if m:
                _, old_version = m.groups()
                break
    assert old_version
    logging.info(f"Drake's old jpeg version: {old_version}")

    # Find out which version upstream is pinned to.
    new_version = None
    with open(local / "third_party/jpeg/workspace.bzl", encoding="utf-8") as f:
        for line in f.readlines():
            m = re.search(r"archive/refs/tags/([0-9a-z.]*).tar.gz", line)
            if m:
                (new_version,) = m.groups()
                break
    assert new_version
    logging.info(f"Drake's new jpeg version: {new_version}")

    # Bail out early.
    if new_version == old_version:
        logging.info("No new archive is required")
        return

    # Update the repository.bzl.
    logging.info("Updating repository.bzl ...")
    with open(mydir / "repository.bzl", encoding="utf-8") as f:
        old_repository_bzl = f.read()
    new_repository_bzl = old_repository_bzl.replace(old_version, new_version)
    assert new_repository_bzl != old_repository_bzl
    new_repository_bzl = re.sub(
        r'"[0-9a-f]{64}"',
        '""',
        new_repository_bzl,
    )
    with open(mydir / "repository.bzl", "w", encoding="utf-8") as f:
        f.write(new_repository_bzl)
    logging.warning("Be sure to fix the sha256sum manually!")


assert __name__ == "__main__"
_main()
