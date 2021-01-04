import argparse
import os
from os.path import abspath, isabs, isdir, join
from pathlib import Path
from shutil import copy, rmtree
from subprocess import check_call
import sys


def _die(s):
    print(s, file=sys.stderr)
    exit(1)


def _minify(input_dir):
    # List of full paths to .js plugins
    plugins = []
    if plugins:
        plugins_min_js = os.path.join(input_dir, "js", "plugins-min.js")
        check_call(
            [
                "uglifyjs "
                + " ".join(plugins)
                + " -o " + plugins_min_js + " -c -m"
            ], shell=True
        )


def gen_main(input_dir):
    """Main entry point for generation.

    Args:
        input_dir: Directory which contains initial input files.
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_dir", type=str, required=True,
        help="Output directory. Does not have to exist beforehand.")
    args = parser.parse_args()
    out_dir = args.out_dir
    if out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "doc")
    if not isabs(out_dir):
        _die("--out_dir must be absolute path: {}".format(out_dir))

    _minify(input_dir)

    # Generate.
    check_call(
        [
            "jekyll build -s " + input_dir + " -d " + out_dir,
        ], shell=True
    )


def preview_main(input_dir):
    """Main entry point for preview
    """
    # Choose an arbitrary location for generating documentation.
    out_dir = abspath("jekyll-tmp")
    if isdir(out_dir):
        rmtree(out_dir)

    _minify(input_dir)

    # Generate.
    check_call(
        [
            "jekyll serve -s " + input_dir + " -d " + out_dir,
        ], shell=True
    )
