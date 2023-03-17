"""Downloads, verifies, and extracts a remote archive file. Do not run this
program by hand; it is used by drake/multibody/parsing/package_map.cc to fetch
remote packages on demand into a per-user local cache (e.g., ~/.cache/...).

WARNING: This tool will recursively delete (and recreate) the `output_dir` as
well as a few files and directories with a similar name but added extension.
For example, with an output_dir of `~/.cache/drake/package_map/foo` the tool
will delete and/or overwrite these files and/or directories:

 ~/.cache/drake/package_map/foo
 ~/.cache/drake/package_map/foo.tmp
 ~/.cache/drake/package_map/foo.README
"""

# N.B. The ONLY packages we're allowed to use here are the Python standard
# library. This program is run using the host OS's built-in Python interpreter,
# which might not offer anything more.
import hashlib
import json
import logging
from pathlib import Path
import os
import urllib.parse
import urllib.request as request
import shutil
import sys
import tempfile
from typing import List


def _fail(message):
    logging.getLogger("package_downloader").error(message)
    sys.exit(1)


def _run(*, temp_dir: Path, package_name: str, urls: List[str], sha256: str,
         output_dir: Path, archive_type: str = None, strip_prefix: str = None):
    """Runs the download and extract logic, assuming already-validated args.

    Args:
        temp_dir: A path to a private (and empty) scratch directory. The caller
            is responsible for cleaning it up; we don't need to.
        package_name: The ROS package name for this package.
        urls: A list of download URLs for this package. Each one will be tried
            in turn until one succeeds.
        sha256: The cryptographic checksum of the file to be downloaded, as a
            64-character hexadecimal string.
        output_dir: The root directory to extract into. Its parent must exist,
            but the output_dir itself generally will not already exist. This
            script will create the output_dir if and only if the download and
            extraction was successful, i.e., as a single atomic transaction.
        archive_type: The archive type of the downloaded file. When not given,
            the archive type is determined from the file extension of the URL.
        strip_prefix: A directory prefix to strip from the extracted files. If
            there are files outside of this directory, they will be discarded
            and inaccessible.
    """
    # Add a README nearby to help explain the fetched data.
    # To avoid stomping on concurrently-running download of the same package
    # from another process, we need to write into a tempfile and then rename
    # it into place as a final step. (Renames are atomic; writes are not.)
    readme = output_dir.with_name(output_dir.name + ".README")
    fd, temp = tempfile.mkstemp(
        dir=readme.parent,
        prefix=f"temp.{readme.name}.")
    temp = Path(temp)
    with os.fdopen(fd, "w") as f:
        f.write(f"Our sibling directory ./{output_dir.name} contains "
                f"package://{package_name}.\n\n"
                f"It was downloaded by Drake from one of these URLs:\n")
        for url in urls:
            f.write(f" {url}\n")
    temp.chmod(0o644)  # Upgrade permissions to 'rw-r--r--' (from 'rw-------').
    temp.rename(readme)

    # Try each url in turn.
    success = False
    errors = []
    for url in urls:
        basename = urllib.parse.urlparse(url).path.split("/")[-1] or "empty"
        temp_filename = temp_dir / basename
        hasher = hashlib.sha256()
        with open(temp_filename, "wb") as f:
            try:
                with request.urlopen(url=url, timeout=30) as response:
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
            success = True
            break
        errors.append(
            f"Candidate {url} failed:\n"
            f"Checksum mismatch; was {download_sha256} but wanted {sha256}.")

    # Report in case no downloads succeeded.
    if not success:
        messages = "\n\n".join(errors)
        _fail(f"All downloads failed:\n\n{messages}")

    # Unpack and check that the strip_prefix was valid.
    unpack_dir = temp_dir / "unpack"
    shutil.unpack_archive(filename=temp_filename, extract_dir=unpack_dir,
                          format=archive_type)
    unpack_package_dir = unpack_dir / (strip_prefix or "")
    if not unpack_package_dir.is_dir():
        _fail(f"The strip_prefix='{strip_prefix}' does not exist "
              f"within {basename}")

    # Now comes the really tricky part. Moving a directory is atomic but only
    # on the same filesystem! We need to move it from the temporary directory
    # into the output directory (under a temporary name), and then atomically
    # rename it back to the real name.
    with tempfile.TemporaryDirectory(
            dir=output_dir.parent,
            prefix=f"temp.{output_dir.name}.") as output_dir_temp:
        move_destination = Path(output_dir_temp) / "incoming"
        shutil.move(src=unpack_package_dir, dst=move_destination)
        try:
            move_destination.rename(output_dir)
        except OSError:
            # This can happen if some *other* downloader process succeeded with
            # its download & extract & move & rename while we were still only
            # partway done. If that occurs, then the other downloader succeeded
            # and we can declare "mission accomplished" for ourself, too.
            pass

    # Double-check that we've met our victory condition: the output_dir that we
    # were supposed to create using the downloaded archive actually exists now.
    assert output_dir.is_dir()


def _main(argv):
    # Read our config file.
    config_json, = argv
    with open(config_json, "r") as f:
        kwargs = json.load(f)

    # Sanity check the required arguments. We use assert (not exceptions)
    # because the package_map.cc code should guarantee these always pass.
    package_name = kwargs["package_name"]
    urls = kwargs["urls"]
    sha256 = kwargs["sha256"]
    output_dir = kwargs["output_dir"]
    assert len(package_name) > 0
    assert len(urls) > 0
    assert len(sha256) == 64
    int(f"0x{sha256}", 16)
    assert "/drake/package_map/" in output_dir, output_dir
    kwargs["output_dir"] = Path(output_dir).resolve()

    # Download and extract.
    with tempfile.TemporaryDirectory(prefix="drake_downloader_") as temp_dir:
        _run(temp_dir=Path(temp_dir), **kwargs)


if __name__ == "__main__":
    logging.basicConfig()
    _main(sys.argv[1:])
