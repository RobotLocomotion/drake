"""Mirrors source archives used by repository rules to the drake-mirror bucket
on Amazon S3. Unless either --no-download or --no-upload option is
specified, needs suitable AWS credentials to be configured per
https://boto3.amazonaws.com/v1/documentation/api/latest/guide/configuration.html.

This script is neither called during the build nor expected to be called by
most developers or users of the project. It is only supported when run under
Python 3 on macOS or Ubuntu 18.04 (Bionic Beaver).

To run:

  bazel build --config=python3 //tools/workspace:mirror_to_s3
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
    for value in read_repository_metadata().values():
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
                           'Multiple non-mirror urls. Verify BUCKET_URL and '
                           'CLOUDFRONT_URL are correct and check for other '
                           'duplicate entries.')
                    transformed_value['url'] = url
            transformed_metadata.append(transformed_value)
    s3_resource = boto3.resource('s3')
    for value in transformed_metadata:
        if '--no-download' in argv:
            print('Not querying S3 object key {} '
                  'because --no-download was specified'.format(
                      value['object_key']))
            continue
        s3_object = s3_resource.Object(BUCKET_NAME, value['object_key'])
        try:
            s3_object.load()
            print('S3 object key {} already exists'.format(
                value['object_key']))
        except botocore.exceptions.ClientError as exception:
            # https://docs.aws.amazon.com/AmazonS3/latest/API/RESTObjectHEAD.html#rest-object-head-permissions
            if exception.response['Error']['Code'] in ['403', '404']:
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
                    if '--no-upload' in argv:
                        print('Not uploading to S3 object key '
                              '{} because --no-upload was specified...'.format(
                                  value['object_key']))
                    else:
                        print('Uploading to S3 object key {}...'.format(
                            value['object_key']))
                        s3_object.upload_file(filename)
            else:
                raise


if __name__ == '__main__':
    main(sys.argv)
