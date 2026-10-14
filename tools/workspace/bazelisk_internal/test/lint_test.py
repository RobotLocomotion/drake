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

Additionally, you must update the bazelisk entries in
setup/ubuntu/packages.json to match the new version, including the urls and
sha256 checksums. To calculate a new checksum, download the deb file and use:
    shasum -a 256 'xxx.deb'
************************************************************************
"""


class BazeliskLintTest(unittest.TestCase):
    def _read(self, respath):
        """Returns the contents of the given resource path."""
        manifest = runfiles.Create()
        path = Path(manifest.Rlocation(respath))
        return path.read_text(encoding="utf-8")

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
        are pinned to the same release as the repository.
        """
        metadata = json.loads(
            self._read("bazelisk_internal/drake_repository_metadata.json")
        )
        commit = metadata["commit"]
        version = commit.removeprefix("v")
        packages = json.loads(self._read("drake/setup/ubuntu/packages.json"))
        bazelisk_packages = [p for p in packages if p["name"] == "bazelisk"]
        self.assertTrue(bazelisk_packages)
        for package in bazelisk_packages:
            self.assertEqual(package["version"], version, _HOW_TO_FIX)
            for url in package["urls"]:
                self.assertIn(f"/{commit}/", url, _HOW_TO_FIX)
