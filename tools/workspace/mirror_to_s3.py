"""Mirrors source archives used by repository rules to the drake-mirror bucket
on Amazon S3. Unless either --no-download or --no-upload option is
specified, needs suitable AWS credentials to be configured per
https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html.

This script is neither called during the build nor expected to be called by
most developers or users of the project. It is only supported when run under
Python 3 on macOS or Ubuntu 18.04 (Bionic Beaver).

To run:

  bazel build //tools/workspace:mirror_to_s3
  bazel-bin/tools/workspace/mirror_to_s3 [--no-download] [--no-upload]

The --no-download option implies --no-upload.
"""

import hashlib
import os
import sys
import tempfile

import boto3
import botocore
import requests

from drake.tools.workspace.metadata import read_repository_metadata


BUCKET_NAME = 'drake-mirror'
BUCKET_URL = 'https://s3.amazonaws.com/drake-mirror/'
CLOUDFRONT_URL = 'https://drake-mirror.csail.mit.edu/'
CHUNK_SIZE = 65536


def main(argv):
    transformed_metadata = []
    for key, value in read_repository_metadata().items():
        if 'downloads' in value:
            downloads = value['downloads']
        else:
            downloads = [value]
        for download in downloads:
            transformed_value = {'sha256': download['sha256']}
            for url in download['urls']:
                if url.startswith(BUCKET_URL):
                    transformed_value['object_key'] = url[len(BUCKET_URL):]
                elif not url.startswith(CLOUDFRONT_URL):
                    if 'url' in transformed_value:
                        raise Exception(
                            f'Multiple non-mirror urls for @{key}. Verify '
                            f'BUCKET_URL {BUCKET_URL} and CLOUDFRONT_URL '
                            f'{CLOUDFRONT_URL} are correct and check for '
                            f'duplicate url values.')
                    transformed_value['url'] = url
            if 'object_key' not in transformed_value:
                raise Exception(
                    f'Could NOT determine S3 object key for @{key}. Verify '
                    f'BUCKET_URL {BUCKET_URL} is correct and check for '
                    f'missing url value with prefix {BUCKET_URL}.')
            if 'url' not in transformed_value:
                raise Exception(
                    f'Missing non-mirror url for @{key}. Verify BUCKET_URL '
                    f'{BUCKET_URL} is correct and check for missing url value '
                    f'with prefix {BUCKET_URL}.')
            transformed_metadata.append(transformed_value)
    s3_resource = boto3.resource('s3')
    for value in transformed_metadata:
        object_key = value['object_key']
        sha256 = value['sha256']
        url = value['url']
        if '--no-download' in argv:
            print(f'NOT querying S3 object key {object_key} because '
                  f'--no-download was specified')
            continue
        s3_object = s3_resource.Object(BUCKET_NAME, object_key)
        try:
            s3_object.load()
            print(f'S3 object key {object_key} already exists')
        except botocore.exceptions.ClientError as exception:
            # https://docs.aws.amazon.com/AmazonS3/latest/API/RESTObjectHEAD.html#rest-object-head-permissions
            if exception.response['Error']['Code'] in ['403', '404']:
                print(f'S3 object key {object_key} does NOT exist')
                with tempfile.TemporaryDirectory() as directory:
                    filename = os.path.join(directory,
                                            os.path.basename(object_key))
                    print(f'Downloading from URL {url}...')
                    with requests.get(url, stream=True) as response:
                        with open(filename, 'wb') as file_object:
                            for chunk in response.iter_content(
                                    chunk_size=CHUNK_SIZE):
                                file_object.write(chunk)
                    print(f'Computing and verifying SHA-256 checksum of '
                          f'file {filename}...')
                    hash_object = hashlib.sha256()
                    with open(filename, 'rb') as file_object:
                        buffer = file_object.read(CHUNK_SIZE)
                        while buffer:
                            hash_object.update(buffer)
                            buffer = file_object.read(CHUNK_SIZE)
                    hexdigest = hash_object.hexdigest()
                    if hexdigest != sha256:
                        raise Exception(
                            f'Expected SHA-256 checksum of file {filename} to '
                            f'be {sha256}, but actual checksum was computed '
                            f'to be {hexdigest}')
                    if '--no-upload' in argv:
                        print(f'NOT uploading file {filename} to S3 object '
                              f'key {object_key} because --no-upload was '
                              f'specified')
                    else:
                        print(f'Uploading file {filename} to S3 object key '
                              f'{object_key}...')
                        s3_object.upload_file(filename)
            else:
                raise


if __name__ == '__main__':
    main(sys.argv)
