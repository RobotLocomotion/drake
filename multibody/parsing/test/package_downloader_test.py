import multibody.parsing.package_downloader as mut  # ruff: isort: skip

import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from urllib.error import HTTPError
import zipfile

# We'll mock out the internet-touching methods. We don't want our unit test
# touching the internet. See setUp and tearDown below for details.
assert hasattr(mut, "request")
setattr(mut, "request", None)


class TestDownloader(unittest.TestCase):
    def setUp(self):
        # Prepare the test stub flavor of `~/.cache/drake/package_map/`.
        self._scratch_dir = Path(tempfile.mkdtemp())
        self._output_parent_dir = self._scratch_dir / "drake/package_map"
        self._output_parent_dir.mkdir(parents=True, exist_ok=True)

        # Stub our urllib.request in our downloader (the module under test).
        mut.request = self
        self._url_contents = dict()
        self._opened_urls = []

    def tearDown(self):
        mut.request = None

    def urlopen(self, *, url, timeout):
        """When the module under test tries to download from the internet, this
        test stub method will be used instead, like `urllib.request.urlopen`.
        """
        assert timeout > 0
        self._opened_urls.append(url)
        data = self._url_contents.get(url, None)
        if data is None:
            raise HTTPError(url=url, code=404, msg="404", hdrs=None, fp=None)
        return io.BytesIO(data)

    def _checksum(self, data):
        """Returns the sha256 hash of the given data."""
        hasher = hashlib.sha256()
        hasher.update(data)
        return hasher.hexdigest()

    def _create_sample_zip(self):
        """Creates a sample archive, and returns it as (data, checksum)."""
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "a") as z:
            z.writestr("hello/world", b"Hello, world!")
        result = buffer.getvalue()
        return result, self._checksum(result)

    def _call_main(self, expect_success=True, **kwargs):
        """Calls the module under test using the given kwargs (as json)."""
        # Match output_dir to the sha256, as is convention for the downloader.
        output_dir = self._output_parent_dir / kwargs["sha256"]
        kwargs["output_dir"] = str(output_dir)

        # Create the arguments json file.
        json_filename = self._scratch_dir / "kwargs.json"
        with open(json_filename, "w") as f:
            f.write(json.dumps(kwargs))

        # Prepare the error filename.
        error_filename = self._scratch_dir / "error.txt"

        # Wrap a call to main.
        try:
            mut._main([json_filename, error_filename, "UNUSED_ARGUMENT"])
            returncode = 0
        except SystemExit as e:
            returncode = e.code
        self.assertEqual(returncode, 0 if expect_success else 1)

        # Confirm that the downloader did (or did not) create the directory.
        self.assertEqual(returncode == 0, output_dir.exists())

        # Provide our caller with any log message.
        with open(error_filename, "r", encoding="utf-8") as f:
            errors = f.read()
        if returncode == 0:
            self.assertEqual(errors, "")
            return None
        else:
            return errors

    def test_vanilla(self):
        """Sanity checks the most typical control flow."""
        # Create some sample data.
        url = "http://127.0.0.1/example.zip"
        data, sha256 = self._create_sample_zip()
        self._url_contents[url] = data

        # Call the module under test.
        self._call_main(package_name="some_name", urls=[url], sha256=sha256)

        # Check that the downloader requested the proper URL.
        self.assertEqual(self._opened_urls, [url])

        # Check that the downloader created the proper README.
        output_dir = self._output_parent_dir / sha256
        with open(f"{output_dir}.README", encoding="utf-8") as f:
            readme = f.read()
        self.assertIn("some_name", readme)
        self.assertIn(url, readme)

        # Check that the downloader created the proper output file.
        with open(f"{output_dir}/hello/world", encoding="utf-8") as f:
            hello = f.read()
        self.assertEqual(hello, "Hello, world!")

    def test_checksum_failure(self):
        """Checks that sha256 verification is happening."""
        # Create some sample data.
        url = "http://127.0.0.1/example.zip"
        data, _ = self._create_sample_zip()
        self._url_contents[url] = data
        sha256 = "0" * 64

        # Call the module under test.
        errors = self._call_main(
            expect_success=False,
            package_name="some_name",
            urls=[url],
            sha256=sha256,
        )
        self.assertIn("Checksum mismatch", errors)

    def test_archive_type_failure(self):
        """Checks that archive_type is obeyed (via an error when set wrong)."""
        # Create some sample data.
        url = "http://127.0.0.1/example.zip"
        data, sha256 = self._create_sample_zip()
        self._url_contents[url] = data

        # This is wrong; the example data is a zip file.
        archive_type = "gztar"

        # Call the module under test.
        errors = self._call_main(
            expect_success=False,
            package_name="some_name",
            urls=[url],
            sha256=sha256,
            archive_type=archive_type,
        )
        self.assertIn("example.zip", errors)
        self.assertIn("tar file", errors)

    def test_strip_prefix(self):
        """Sanity checks the strip_prefix feature."""
        # Create some sample data.
        url = "http://127.0.0.1/example.zip"
        data, sha256 = self._create_sample_zip()
        self._url_contents[url] = data

        # Call the module under test.
        self._call_main(
            package_name="some_name",
            urls=[url],
            sha256=sha256,
            strip_prefix="hello",
        )

        # Check that the downloader stripped off "hello".
        output_dir = self._output_parent_dir / sha256
        with open(f"{output_dir}/world", encoding="utf-8") as f:
            hello = f.read()
        self.assertEqual(hello, "Hello, world!")

    def test_strip_prefix_failure(self):
        """Checks the error report for a malformed strip_prefix."""
        # Create some sample data.
        url = "http://127.0.0.1/example.zip"
        data, sha256 = self._create_sample_zip()
        self._url_contents[url] = data

        # Call the module under test.
        errors = self._call_main(
            expect_success=False,
            package_name="some_name",
            urls=[url],
            sha256=sha256,
            strip_prefix="wrong",
        )
        self.assertIn("strip_prefix", errors)

        # Now use the correct prefix. This exercises the code path where we
        # need to clean up the output_dir_tmp from a prior unpacking attempt.
        self._call_main(
            package_name="some_name",
            urls=[url],
            sha256=sha256,
            strip_prefix="hello",
        )

    def test_all_urls_failed(self):
        """Checks the error report when all URLs have gone AWOL."""
        # Make some URLs, but don't populate self._url_contents with them.
        # This will produce http 404 errors.
        urls = [
            "http://127.0.0.1/missing.zip",
            "http://127.0.0.2/missing.zip",
            "http://127.0.0.3/missing.zip",
        ]
        sha256 = "0" * 64

        # Call the module under test.
        errors = self._call_main(
            expect_success=False,
            package_name="some_name",
            urls=urls,
            sha256=sha256,
        )
        self.assertIn("All downloads failed", errors)
        for url in urls:
            self.assertIn(url, errors)
