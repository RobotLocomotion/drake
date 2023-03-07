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
import os.path
import urllib.parse
import urllib.request as request
import shutil
import sys
import tempfile


def _fail(message):
    logging.getLogger("package_downloader").error(message)
    sys.exit(1)


def _run(*, temp_dir, package_name, urls, sha256, output_dir,
         archive_type=None, strip_prefix=None):
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
            but the output_dir itself must not already exist. This script will
            create the output_dir only if the download and extraction was
            successful, i.e., as a single atomic transaction.
        archive_type: The archive type of the downloaded file. When not given,
            the archive type is determined from the file extension of the URL.
        strip_prefix: A directory prefix to strip from the extracted files. If
            there are files outside of this directory, they will be discarded
            and inaccessible.
    """
    # Add a README nearby to help explain the fetched data.
    with open(f"{output_dir}.README", "wt", encoding="utf-8") as f:
        f.write(f"Our sibling directory ./{os.path.basename(output_dir)} "
                f"contains package://{package_name}.\n\n")
        f.write("It was downloaded by Drake from one of these URLs:\n")
        for url in urls:
            f.write(f" {url}\n")

    # Erase any spurious lingering data from a prior attempt at extraction.
    output_dir_tmp = f"{output_dir}.tmp"
    if os.path.exists(output_dir_tmp):
        shutil.rmtree(output_dir_tmp)

    # Try each url in turn.
    success = False
    errors = []
    for url in urls:
        basename = urllib.parse.urlparse(url).path.split("/")[-1] or "empty"
        temp_filename = os.path.join(temp_dir, basename)
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
        os.remove(temp_filename)
        errors.append(
            f"Candidate {url} failed:\n"
            f"Checksum mismatch; was {download_sha256} but wanted {sha256}.")

    # Report in case no downloads succeeded.
    if not success:
        messages = "\n\n".join(errors)
        _fail(f"All downloads failed:\n\n{messages}")

    # Unpack and atomically move into place.
    shutil.unpack_archive(filename=temp_filename, extract_dir=output_dir_tmp,
                          format=archive_type)
    src = os.path.join(output_dir_tmp, strip_prefix or "")
    if not os.path.exists(src):
        _fail(f"The strip_prefix='{strip_prefix}' does not exist "
              f"within {output_dir_tmp}")
    os.rename(src=src, dst=output_dir)
    try:
        shutil.rmtree(output_dir_tmp)
    except OSError:
        pass


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
    assert not os.path.exists(output_dir), output_dir
    kwargs["output_dir"] = os.path.realpath(output_dir)

    # Download and extract.
    with tempfile.TemporaryDirectory(prefix="drake_downloader_") as temp_dir:
        _run(temp_dir=temp_dir, **kwargs)


if __name__ == "__main__":
    logging.basicConfig()
    _main(sys.argv[1:])
