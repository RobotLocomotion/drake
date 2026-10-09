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
    tools/workspace/bazelisk_debs_internal/repository.bzl
and adjust the expected checksums accordingly.
To calculate a new checksum, download the deb file specifed in the json and use:
    shasum -a 256 'xxx.deb'
************************************************************************
"""


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

    def test_repository_versions(self):
        """Checks that the source and deb repositories are pinned to the same
        release.
        """
        source_commit = self._read_metadata("bazelisk_internal")["commit"]
        debs_commit = self._read_metadata("bazelisk_debs_internal")["commit"]
        self.assertEqual(source_commit, debs_commit, _HOW_TO_FIX)

    def test_setup_packages(self):
        """Checks that our files to download as listed in
        setup/ubuntu/packages.json are up to date with the repository
        attachments.
        """
        metadata = self._read_metadata("bazelisk_debs_internal")
        version = metadata["commit"].removeprefix("v")
        attachments = metadata["attachments"]
        download_urls = {
            url.split("/")[-1]: download["urls"]
            for download in metadata["downloads"]
            for url in download["urls"]
        }
        packages = json.loads(self._read("drake/setup/ubuntu/packages.json"))
        bazelisk_packages = [p for p in packages if p["name"] == "bazelisk"]
        self.assertTrue(bazelisk_packages)
        for package in bazelisk_packages:
            self.assertEqual(package["version"], version, _HOW_TO_FIX)
            for url in package["urls"]:
                basename = url.split("/")[-1]
                self.assertIn(basename, attachments, _HOW_TO_FIX)
                self.assertEqual(
                    package["sha256"], attachments[basename], _HOW_TO_FIX
                )
                self.assertIn(url, download_urls[basename], _HOW_TO_FIX)
