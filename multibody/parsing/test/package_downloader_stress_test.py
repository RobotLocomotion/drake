import glob
import hashlib
import io
import json
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
        scratch_dir = Path(tempfile.mkdtemp())

        # Create a sample archive.
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "a") as z:
            z.writestr("hello/world", b"Hello, world!")
        data = buffer.getvalue()
        sha256 = hashlib.sha256(data).hexdigest()
        archive = scratch_dir / "archive.zip"
        with open(archive, "wb") as f:
            f.write(data)
        url = "file://" + str(archive)

        # Prepare the test stub flavor of `~/.cache/drake/package_map/`.
        output_dir = scratch_dir / "drake" / "package_map" / sha256
        output_dir.parent.mkdir(parents=True)

        # Create the arguments json file.
        kwargs = scratch_dir / "kwargs.json"
        with open(kwargs, "w", encoding="utf-8") as f:
            f.write(
                json.dumps(
                    dict(
                        package_name="some_name",
                        output_dir=str(output_dir),
                        urls=[url],
                        sha256=sha256,
                    )
                )
            )

        # Call the downloader program repeatedly and in parallel.
        N = 100
        cpus = 8
        children = []
        launch_count = 0
        while True:
            # Launch child processes until we are running at max parallelism.
            # In theory, the scheduler does not *guarantee* the subprocesses
            # will run in parallel, but in practice they do and any concurrency
            # errors in the downloader show up very quickly, usually within the
            # first handful.
            while (len(children) + 1 < cpus) and (launch_count < N):
                error_filename = scratch_dir / f"error-{launch_count:03}.txt"
                children.append(
                    subprocess.Popen(
                        [
                            sys.executable,
                            "multibody/parsing/package_downloader.py",
                            kwargs,
                            error_filename,
                            "UNUSED_ARGUMENT",
                        ]
                    )
                )
                launch_count += 1
            # Reap completed processes.
            for i in reversed(range(len(children))):
                returncode = children[i].poll()
                if returncode is not None:
                    self.assertEqual(returncode, 0)
                    del children[i]
            # Once we're idle and have reached max launch_count, we're done.
            if len(children) == 0 and launch_count >= N:
                break

        # Confirm that the archive was unpacked and no temporary files remain.
        found = [
            str(Path(x).relative_to(scratch_dir))
            for x in glob.glob(f"{scratch_dir}/drake/**", recursive=True)
        ]
        self.assertListEqual(
            sorted(found),
            [
                "drake",
                "drake/package_map",
                f"drake/package_map/{sha256}",
                f"drake/package_map/{sha256}.README",
                f"drake/package_map/{sha256}/hello",
                f"drake/package_map/{sha256}/hello/world",
            ],
        )
