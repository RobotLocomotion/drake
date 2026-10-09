"""Mirrors source archives used by repository rules, as well as the downloaded
packages listed in setup/ubuntu/packages.json, to the drake-mirror bucket on
Amazon S3.

Unless either --no-download or --no-upload option is specified, needs suitable
AWS credentials to be configured per
https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html.

This script is neither called during the build nor expected to be called by
most developers or users of the project. It is only supported when run under
Python 3 on macOS or Ubuntu.

To run:

  bazel build //tools/workspace:mirror_to_s3
  bazel-bin/tools/workspace/mirror_to_s3 [--no-download] [--no-upload]

The --no-download option implies --no-upload.
"""

import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any

import boto3
import botocore
from python import runfiles
import requests

from tools.workspace.metadata import read_repository_metadata

BUCKET_NAME = "drake-mirror"
BUCKET_URL = "https://s3.amazonaws.com/drake-mirror/"
CLOUDFRONT_URL = "https://drake-mirror.csail.mit.edu/"
CHUNK_SIZE = 65536
UBUNTU_PACKAGES_PATH = "drake/setup/ubuntu/packages.json"


def _mirror_prefix(url: str) -> str | None:
    """Returns the drake-mirror prefix of the given url, or None if the url
    does not point to our mirror.
    """
    for prefix in (BUCKET_URL, CLOUDFRONT_URL):
        if url.startswith(prefix):
            return prefix
    return None


def _read_ubuntu_packages() -> dict[str, Any]:
    """Returns the list of package dicts from setup/ubuntu/packages.json."""
    manifest = runfiles.Create()
    json_path = Path(manifest.Rlocation(UBUNTU_PACKAGES_PATH))
    return json.loads(json_path.read_text(encoding="utf-8"))


def _transform_download(label: str, download: dict[str, Any]) -> dict[str, str]:
    """Given a download dict with "sha256" and "urls", returns a dict with the
    "sha256", the S3 "object_key" to mirror into, and the upstream "url" to
    mirror from.
    """
    transformed_value = {"sha256": download["sha256"]}
    for url in download["urls"]:
        prefix = _mirror_prefix(url)
        if prefix is not None:
            transformed_value.setdefault("object_key", url[len(prefix) :])
        else:
            if "url" in transformed_value:
                raise RuntimeError(
                    f"Multiple non-mirror urls for {label}. Verify "
                    f"BUCKET_URL {BUCKET_URL} and CLOUDFRONT_URL "
                    f"{CLOUDFRONT_URL} are correct and check for "
                    f"duplicate url values."
                )
            transformed_value["url"] = url
    if "object_key" not in transformed_value:
        raise RuntimeError(
            f"Could NOT determine S3 object key for {label}. Verify "
            f"BUCKET_URL {BUCKET_URL} and CLOUDFRONT_URL {CLOUDFRONT_URL} "
            f"are correct and check for missing url value with either prefix."
        )
    if "url" not in transformed_value:
        raise RuntimeError(
            f"Missing non-mirror url for {label}. Verify BUCKET_URL "
            f"{BUCKET_URL} and CLOUDFRONT_URL {CLOUDFRONT_URL} are correct "
            f"and check for missing url value without either prefix."
        )
    return transformed_value


def _mirror_to_s3(
    s3_resource, value: dict[str, Any], argv: list[str] | None
) -> None:
    """Given a dict as returned by _transform_download, uploads the file to S3
    unless it is already present there.
    """
    object_key = value["object_key"]
    sha256 = value["sha256"]
    url = value["url"]
    if "--no-download" in argv:
        print(
            f"NOT querying S3 object key {object_key} because "
            f"--no-download was specified"
        )
        return
    s3_object = s3_resource.Object(BUCKET_NAME, object_key)
    try:
        s3_object.load()
        print(f"S3 object key {object_key} already exists")
        return
    except botocore.exceptions.ClientError as exception:
        # https://docs.aws.amazon.com/AmazonS3/latest/API/RESTObjectHEAD.html#rest-object-head-permissions
        if exception.response["Error"]["Code"] not in ["403", "404"]:
            raise
    print(f"S3 object key {object_key} does NOT exist")
    with tempfile.TemporaryDirectory() as directory:
        filename = os.path.join(directory, os.path.basename(object_key))
        print(f"Downloading from URL {url}...")
        with (
            requests.get(url, stream=True) as response,
            open(filename, "wb") as file_object,
        ):
            file_object.writelines(response.iter_content(chunk_size=CHUNK_SIZE))
        print(f"Computing and verifying SHA-256 checksum of file {filename}...")
        hash_object = hashlib.sha256()
        with open(filename, "rb") as file_object:
            buffer = file_object.read(CHUNK_SIZE)
            while buffer:
                hash_object.update(buffer)
                buffer = file_object.read(CHUNK_SIZE)
        hexdigest = hash_object.hexdigest()
        if hexdigest != sha256:
            raise RuntimeError(
                f"Expected SHA-256 checksum of file {filename} to be "
                f"{sha256}, but actual checksum was computed to be "
                f"{hexdigest}"
            )
        if "--no-upload" in argv:
            print(
                f"NOT uploading file {filename} to S3 object key "
                f"{object_key} because --no-upload was specified"
            )
            return
        print(f"Uploading file {filename} to S3 object key {object_key}...")
        s3_object.upload_file(filename)


def main(argv: list[str] | None = None) -> None:
    transformed_metadata = []
    for key, value in read_repository_metadata().items():
        if not value.get("mirror_to_s3", True):
            continue
        rule_type = value["repository_rule_type"]
        if rule_type in ("alias", "pkg_config"):
            continue
        if "downloads" in value:
            downloads = value["downloads"]
        else:
            downloads = [value]
        for download in downloads:
            transformed_metadata.append(
                _transform_download(f"@{key}", download)
            )
    for package in _read_ubuntu_packages():
        # Packages whose only urls are our mirror were uploaded by hand; there
        # is no upstream to mirror from.
        if all(_mirror_prefix(url) for url in package["urls"]):
            continue
        arches = ",".join(package["arches"])
        label = f"{package['name']} ({arches}) in setup/ubuntu/packages.json"
        transformed_metadata.append(_transform_download(label, package))
    s3_resource = boto3.resource("s3")
    for value in transformed_metadata:
        _mirror_to_s3(s3_resource, value, argv)


if __name__ == "__main__":
    main(sys.argv)
