import json
from pathlib import Path
import unittest

from python import runfiles

_HOW_TO_FIX = """\n
If you're seeing this, the automated upgrade in
tools/workspace/bazelisk_internal probably has a bug, or somehow its changes
got lost. For now, here are the manual instructions...

************************************************************************
To update Drake's vendored copy of bazelisk:

$ cd drake
$ bazel build @bazelisk_internal//:*
$ cp -t third_party/com_github_bazelbuild_bazelisk/ \\
    bazel-drake/external/+internal_repositories+bazelisk_internal/LICENSE \\
    bazel-drake/external/+internal_repositories+bazelisk_internal/bazelisk.py

Additionally, you must manually update the version numbers in
    setup/ubuntu/packages.json
and adjust the expected checksums accordingly.
To calculate a new checksum, download the deb file specifed in the json and use:
    shasum -a 256 'xxx.deb'
************************************************************************
"""

_GITHUB_URL = "https://github.com/bazelbuild/bazelisk"
_MIRROR_URL = "https://drake-mirror.csail.mit.edu/github/bazelbuild/bazelisk"


class BazeliskLintTest(unittest.TestCase):
    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

    def _read_metadata(self, repo_name):
        """Returns the repository metadata dict for the given repository."""
        return json.loads(
            self._read(f"{repo_name}/drake_repository_metadata.json")
        )

    def test_vendored_copy(self):
        """Checks that our vendored copy of bazelisk is up to date with the
        repository pin.
        """
        for name in ["LICENSE", "bazelisk.py"]:
            upstream_content = self._read(f"bazelisk_internal/{name}")
            vendored_content = self._read(
                f"drake/third_party/com_github_bazelbuild_bazelisk/{name}"
            )
            self.assertMultiLineEqual(
                upstream_content, vendored_content, _HOW_TO_FIX
            )

    def test_setup_packages(self):
        """Checks that the bazelisk debs listed in setup/ubuntu/packages.json
        are pinned to the same release as the repository, and are downloaded
        from both GitHub and our drake-mirror.
        """
        commit = self._read_metadata("bazelisk_internal")["commit"]
        version = commit.removeprefix("v")
        packages = json.loads(self._read("drake/setup/ubuntu/packages.json"))
        bazelisk_packages = [p for p in packages if p["name"] == "bazelisk"]
        self.assertTrue(bazelisk_packages)
        for package in bazelisk_packages:
            self.assertEqual(package["version"], version, _HOW_TO_FIX)
            (arch,) = package["arches"]
            path = f"{commit}/bazelisk-{arch}.deb"
            expected_urls = [
                f"{_GITHUB_URL}/releases/download/{path}",
                f"{_MIRROR_URL}/{path}",
            ]
            self.assertListEqual(package["urls"], expected_urls, _HOW_TO_FIX)
