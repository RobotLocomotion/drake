import argparse
from http.server import SimpleHTTPRequestHandler
import os
from os import listdir, symlink, mkdir
from os.path import abspath, dirname, isabs, isdir, isfile, join
from shutil import rmtree
from socketserver import TCPServer
from subprocess import check_call
import sys
import tempfile
import webbrowser

_SPHINX_BUILD = "doc/sphinx_build.py"


def _die(s):
    print(s, file=sys.stderr)
    exit(1)


def gen_main(input_dir, strict, src_func=None):
    """Main entry point for generation.

    Args:
        input_dir: Directory which contains initial input files.
        strict: Determines if Sphinx warnings should be interpreted as errors.
        src_func: (optional) Callable of form `f(src_dir)` which will introduce
            additional source files to `src_dir`.
    """
    assert isfile(_SPHINX_BUILD), "Please execute via 'bazel run'"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_dir", type=str, required=True,
        help="Output directory. Does not have to exist beforehand.")
    parser.add_argument(
        "--debug", action="store_true",
        help="If enabled, leaves intermediate files that are otherwise "
             "deleted.")
    args = parser.parse_args()
    out_dir = args.out_dir
    if out_dir == "<test>":
        out_dir = join(os.environ["TEST_TMPDIR"], "doc")
    if not isabs(out_dir):
        _die("--out_dir must be absolute path: {}".format(out_dir))
    # Use default temp directory, handling both `bazel run` and `bazel test`.
    tmp_dir = tempfile.mkdtemp(dir=os.environ.get("TEST_TMPDIR"))
    doctree_dir = join(tmp_dir, "doctrees")
    src_dir = join(tmp_dir, "src")
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


class _Handler(SimpleHTTPRequestHandler):
    # An HTTP handler without logging.
    def log_request(*_):
        pass


def _str2bool(value):
    # From: https://stackoverflow.com/a/19233287/7829525
    return value.lower() in ("yes", "y", "true", "t", "1")


def preview_main(gen_script, default_port):
    """Main entrypoint for previewing documentation.

    Args:
        gen_script: Generation script, required to generate docs.
        default_port: Default port for local HTTP server.
    """
    assert isfile(_SPHINX_BUILD), "Please execute via 'bazel run'"
    parser = argparse.ArgumentParser()
    parser.register('type', 'bool', _str2bool)
    parser.add_argument(
        "--browser", type='bool', default=True, metavar='BOOL',
        help="Open browser. Disable this if you are frequently recompiling.")
    parser.add_argument(
        "--port", type=int, default=default_port, metavar='PORT',
        help="Port for serving doc pages with a HTTP server.")
    args = parser.parse_args()
    # Choose an arbitrary location for generating documentation.
    out_dir = abspath("sphinx-tmp")
    if isdir(out_dir):
        rmtree(out_dir)
    # Generate.
    check_call([sys.executable, gen_script, "--out_dir", out_dir])
    print("Sphinx preview docs are available at:")
    file_url = "file://{}".format(join(out_dir, "index.html"))
    browser_url = file_url
    print()
    print("  {}".format(file_url))
    # Serve the current directory for local browsing. Required for MacOS.
    # N.B. We serve the preview via a HTTP server because it is necessary for
    # certain browsers (Safari on MacOS, possibly Chrome) due to local file
    # restrictions.
    os.chdir(out_dir)
    sockaddr = ("127.0.0.1", args.port)
    TCPServer.allow_reuse_address = True
    httpd = TCPServer(sockaddr, _Handler)
    http_url = "http://{}:{}/index.html".format(*sockaddr)
    print()
    print("  {}".format(http_url))
    # Default to using HTTP serving only on MacOS; on Ubuntu, it can spit
    # out errors and exceptions about broken pipes, 404 files, etc.
    if sys.platform == "darwin":
        browser_url = http_url
    # Try the default browser.
    if args.browser:
        webbrowser.open(browser_url)
    # Wait for server.
    print()
    print("Serving and waiting ... use Ctrl-C to exit.")
    httpd.serve_forever()
