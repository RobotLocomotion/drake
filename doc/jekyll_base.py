import argparse
import os
from os.path import abspath, exists, isabs, isdir, join
from pathlib import Path
from shutil import copy, rmtree
from subprocess import check_call
import sys
import tempfile


def _die(s):
    print(s, file=sys.stderr)
    sys.exit(1)


def _minify(input_dir, out_dir):
    plugin_dir = join(out_dir, "js")
    if not exists(plugin_dir):
        os.mkdir(plugin_dir)

    # drake-specific plugin
    drake_js = join(input_dir, "_js", "drake.js")
    drake_min_js = join(plugin_dir, "drake-min.js")
    check_call(
        [
            "uglifyjs", drake_js,
            "-o", drake_min_js,
            "-c", "-m",
        ]
    )

    # third party plugins
    third_party_dir = join(Path(input_dir).parent, "third_party")
    history_adapter_js = join(
        third_party_dir, "history_js", "history.adapter.jquery.js")
    history_js = join(third_party_dir, "history_js", "history.js")
    waypoint_js = join(third_party_dir, "waypoints", "waypoint.js")

    plugins = [history_adapter_js,
               history_js,
               waypoint_js]

    plugins_min_js = join(plugin_dir, "plugins-min.js")
    check_call(
        [
            "uglifyjs"
        ] + plugins +
        [
            "-o", plugins_min_js,
            "-c", "-m",
        ]
    )


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

    # Minify files directly to output location.
    _minify(input_dir, out_dir)

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
        # Minify files directly to output location.
        _minify(input_dir, out_dir)

        # Generate.
        check_call(
            [
                "jekyll", "serve",
                "-s", input_dir,
                "-d", out_dir,
                "--port", str(args.port),
            ]
        )
