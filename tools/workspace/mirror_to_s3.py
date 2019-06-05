"""Mirrors source archives used by repository rules to the drake-mirror bucket
on Amazon S3. Needs suitable AWS credentials to be configured per
https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html.

This script is neither called during the build nor expected to be called by
most developers or users of the project. It is only supported when run under
Python 3 on macOS or Ubuntu 18.04 (Bionic Beaver).

To run:

  bazel build --config=python3 //tools/workspace:mirror_to_s3
  bazel-bin/tools/workspace/mirror_to_s3
"""

import hashlib
import os
import tempfile

import boto3
import botocore
import requests

from drake.tools.workspace.metadata import read_repository_metadata


BUCKET_NAME = 'drake-mirror'
BUCKET_URL = 'https://s3.amazonaws.com/drake-mirror/'
CLOUDFRONT_URL = 'https://drake-mirror.csail.mit.edu/'
CHUNK_SIZE = 65536


def main():
    transformed_metadata = {}
    for key, value in read_repository_metadata().items():
        transformed_metadata[key] = {'sha256': value['sha256']}
        for url in value['urls']:
            if url.startswith(BUCKET_URL):
                transformed_metadata[key]['object_key'] = url[len(BUCKET_URL):]
            elif not url.startswith(CLOUDFRONT_URL):
                if 'url' in transformed_metadata[key]:
                    raise Exception(
                       'Multiple non-mirror urls. Verify BUCKET_URL and '
                       'CLOUDFRONT_URL are correct and check for other '
                       'duplicate entries.')
                transformed_metadata[key]['url'] = url
    s3_resource = boto3.resource('s3')
    for value in transformed_metadata.values():
        s3_object = s3_resource.Object(BUCKET_NAME, value['object_key'])
        try:
            s3_object.load()
            print('S3 object key {} already exists'.format(
                value['object_key']))
        except botocore.exceptions.ClientError as exception:
            if exception.response['Error']['Code'] == '404':
                print('S3 object key {} does NOT exist'.format(
                    value['object_key']))
                with tempfile.TemporaryDirectory() as directory:
                    filename = os.path.join(
                        directory, os.path.basename(value['object_key']))
                    print('Downloading from URL {}...'.format(value['url']))
                    with requests.get(value['url'], stream=True) as response:
                        with open(filename, 'wb') as file_object:
                            for chunk in response.iter_content(
                                    chunk_size=CHUNK_SIZE):
                                file_object.write(chunk)
                    print('Computing and verifying SHA-256 checksum...')
                    hash_object = hashlib.sha256()
                    with open(filename, 'rb') as file_object:
                        buffer = file_object.read(CHUNK_SIZE)
                        while buffer:
                            hash_object.update(buffer)
                            buffer = file_object.read(CHUNK_SIZE)
                    if hash_object.hexdigest() != value['sha256']:
                        raise Exception('Checksum mismatch')
                    print('Uploading to S3 object key {}...'.format(
                        value['object_key']))
                    s3_object.upload_file(filename)
            else:
                raise


if __name__ == '__main__':
    main()
