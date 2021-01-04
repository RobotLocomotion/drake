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


def _minify(intput_dir, output_dir):
    # TODO(betsymcphail): Once implemented for production, the Jekyll site may
    # use both third party and custom .js plugins (e.g. <input_dir>/js/drake.js).
    # Add the full path to any plugins here.
    plugins = []
    if plugins:
        plugin_dir = os.path.join(out_dir, "js")
        if not os.path.exists(plugin_dir):
            os.mkdir(plugin_dir)
        plugins_min_js = join(plugin_dir, "plugins-min.js")
        check_call(
            [
                "uglifyjs",
                " ".join(plugins),
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
        help="Output directory. Does not have to exist beforehand.")
    args = parser.parse_args()
    out_dir = args.out_dir
    if out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "doc")
    if not isabs(out_dir):
        _die("--out_dir must be absolute path: {}".format(out_dir))

    # Generate.
    check_call(
        [
            "jekyll", "build",
            "-s", input_dir,
            "-d", out_dir,
        ]
    )

    # Minify files directly to output location
    _minify(input_dir, out_dir)


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

    # Choose an arbitrary location for generating documentation.
    out_dir = tempfile.TemporaryDirectory().name

    # Generate.
    check_call(
        [
            "jekyll", "serve",
            "-s", input_dir,
            "-d", out_dir,
            "--port", str(args.port),
        ]
    )

    # Minify files directly to output location
    _minify(input_dir, out_dir)
