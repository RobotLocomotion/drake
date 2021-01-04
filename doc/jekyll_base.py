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


def _minify(input_dir, out_dir):
    # TODO(betsymcphail): Once implemented for production, the Jekyll site may
    #  require third party .js plugins in addition to drake.js.
    # Add the full path to any plugins here.
    drake_js = join(input_dir, "_js", "drake.js")
    plugins = [drake_js]
    plugin_dir = join(out_dir, "js")
    if not exists(plugin_dir):
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
