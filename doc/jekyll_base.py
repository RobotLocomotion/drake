import argparse
import os
from os.path import abspath, exists, isabs, isdir, join
from shutil import rmtree
from subprocess import check_call
import sys
import tempfile


def _die(s):
    print(s, file=sys.stderr)
    sys.exit(1)


def gen_main(input_dir):
    """Main entry point for generation.

    Args:
        input_dir: Directory which contains initial input files.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_dir", type=str, required=True,
        help="Output directory. Must be an absolute path and must not exist.")
    args = parser.parse_args()
    out_dir = args.out_dir
    if out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "jekyll")
    if not isabs(out_dir):
        _die("--out_dir must be absolute path: {}".format(out_dir))
    # Jekyll will remove all of the contents of the output directory without
    # warning. Require an empty directory to avoid accidental deletion.
    if exists(out_dir):
        _die("--out_dir must not exist: {}".format(out_dir))

    # Create output directory.
    os.makedirs(out_dir)

    # Generate.
    check_call(
        [
            "jekyll", "build",
            "-s", input_dir,
            "-d", out_dir,
        ]
    )


def preview_main(input_dir, default_port):
    """Main entry point for preview

     Args:
        default_port: Default port for local server.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port", type=int, default=default_port, metavar='PORT',
        help="Port for serving doc pages on a local server.")
    args = parser.parse_args()

    # Choose an arbitrary, temporary location for generating documentation.
    with tempfile.TemporaryDirectory() as out_dir:
        # Generate.
        check_call(
            [
                "jekyll", "serve",
                "-s", input_dir,
                "-d", out_dir,
                "--port", str(args.port),
            ]
        )
