"""Extracts a tarfile archive.
"""

import argparse
import tarfile
import subprocess

import sys
print(sys.argv[1:])
# exit(10)

parser = argparse.ArgumentParser()
parser.add_argument("archive", type=str)
parser.add_argument("--output_dir", type=str, default='.')
parser.add_argument("--strip_prefix", type=str)
# parser.add_argument("--encoding", type=str, nargs='+', default="ascii")
parser.add_argument("--patch_cmd", type=str)
args = parser.parse_args()

tar = tarfile.open(args.archive, 'r')
# Extract files and symlinks.
members = [member for member in tar.getmembers()
           if member.isfile() or member.issym()]
tar_files = sorted((member.name for member in members))
# Apply path transformations.
# According to https://stackoverflow.com/a/8261083/7829525, we can magically
# alter the `.name` field of a member to change where it gets extracted to.
def filter_members(members):
    for member in members:
        if member.name.startswith(args.strip_prefix):
            member.name = member.name[len(args.strip_prefix):].lstrip('/')
        yield member
# Extract all files.
tar.extractall(path=args.output_dir, members=filter_members(members))
# Execute patch command.
print("+ {}".format(args.patch_cmd))
subprocess.check_call(args.patch_cmd, shell=True)
