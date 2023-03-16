import glob
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import zipfile


class StressTestDownloader(unittest.TestCase):
    """Demonstrate that multiple concurrent downloaders do not step on each
    other's toes by trying to overwrite each other's files.
    """

    def test_many_threads(self):
        # Prepare the test stub flavor of `~/.cache/drake/package_map/`.
        scratch_dir = Path(tempfile.mkdtemp())
        output_parent_dir = Path(f"{scratch_dir}/drake/package_map")
        output_parent_dir.mkdir(parents=True)

        # Create a sample archive.
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "a") as z:
            z.writestr("hello/world", b"Hello, world!")
        data = buffer.getvalue()
        sha256 = hashlib.sha256(data).hexdigest()
        archive = f"{scratch_dir}/archive.zip"
        with open(archive, "wb") as f:
            f.write(data)
        url = "file://" + archive

        # Create the arguments json file.
        output_dir = output_parent_dir / sha256
        kwargs = scratch_dir / "kwargs.json"
        with open(kwargs, "w", encoding="utf-8") as f:
            f.write(json.dumps(dict(
                package_name="some_name", output_dir=str(output_dir),
                urls=[url], sha256=sha256)))

        # Call the downloader program repeatedly and in parallel.
        N = 100
        cpus = 8
        children = []
        launch_count = 0
        while True:
            # Launch child processes until we are running at max parallelism.
            while (len(children) + 1 < cpus) and (launch_count < N):
                children.append(subprocess.Popen([
                    sys.executable, "multibody/parsing/package_downloader.py",
                    kwargs]))
                launch_count += 1
            # Reap completed processes.
            for i in reversed(range(len(children))):
                returncode = children[i].poll()
                if returncode is not None:
                    self.assertEqual(returncode, 0)
                    del children[i]
            # Once return back to idle and reached max launch, we're done.
            if len(children) == 0 and launch_count >= N:
                break

        # Confirm that the archive was unpacked and no temporary files remain.
        found = glob.glob("**", root_dir=scratch_dir / "drake", recursive=True)
        self.assertListEqual(sorted(found), [
            f"package_map",
            f"package_map/{sha256}",
            f"package_map/{sha256}.README",
            f"package_map/{sha256}/hello",
            f"package_map/{sha256}/hello/world",
        ])
