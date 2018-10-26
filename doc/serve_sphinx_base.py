from __future__ import print_function

import argparse
import os
from os import listdir, symlink, mkdir
from os.path import abspath, dirname, isabs, isdir, isfile, join
from shutil import rmtree
from subprocess import check_call
import sys
import webbrowser

_SPHINX_BUILD = "doc/sphinx_build.py"


def str2bool(value):
    # From: https://stackoverflow.com/a/19233287/7829525
    return value.lower() in ("yes", "y", "true", "t", "1")


def die(s):
    print(s, file=sys.stderr)
    exit(1)


def main(input_dir, strict, src_func=None):
    assert isfile(_SPHINX_BUILD), "Please execute via 'bazel run'"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_dir", type=str, default=None)
    parser.add_argument(
        "--debug", action="store_true")
    parser.register('type', 'bool', str2bool)
    parser.add_argument(
        "--browser", type='bool', default=True, metavar='BOOL',
        help="Open browser. Disable this if you are frequently recompiling.")
    args = parser.parse_args()
    out_dir = args.out_dir
    if out_dir is None:
        # Backwards-compatibility: Don't require that users specify
        # `--out_dir`; generate to same location as before.
        out_dir = abspath("sphinx-tmp")
        if isdir(out_dir):
            rmtree(out_dir)
    elif out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "doc")
    if not isabs(out_dir):
        die("--out_dir must be absolute path: {}".format(out_dir))
    if isdir(out_dir):
        die("--out_dir must not exist: {}".format(out_dir))
    mkdir(out_dir)

    tmp_dir = join(out_dir, "_tmp")
    doctree_dir = join(tmp_dir, "doctrees")
    src_dir = join(tmp_dir, "src")
    mkdir(tmp_dir)
    # Symlink inputs to src dir (so that we can also generate doc modules).
    mkdir(src_dir)
    for f in listdir(input_dir):
        src_f = join(src_dir, f)
        symlink(join(input_dir, f), src_f)

    # Optionally generate additional input files as source.
    if src_func:
        src_func(src_dir)

    print("Generating documentation...")
    if strict:
        # Turn warnings into errors; else be quiet.
        warning_args = ["-W", "-N", "-q"]
    else:
        warning_args = [
            "-N", "-Q",  # Be very quiet.
            "-T",  # Traceback (for plugin)
        ]
    check_call([
        sys.executable, _SPHINX_BUILD,
        "-b", "html",  # HTML output.
        "-a", "-E",  # Don't use caching.
        "-d", doctree_dir] + warning_args + [
        src_dir,  # Source dir.
        out_dir,
    ])
    if not args.debug:
        rmtree(tmp_dir)
    else:
        print("DEBUG: Temporary files: {}".format(tmp_dir))
    print("Sphinx preview docs are available at:")
    file_url = "file://{}".format(join(out_dir, "index.html"))
    print("  {}".format(file_url))
    # Try the default browser.
    if args.browser:
        webbrowser.open(file_url)
