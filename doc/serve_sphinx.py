"""
This program serves sphinx.zip for a web browser.

Run this via:
  $ bazel run //doc:serve_sphinx
"""

from __future__ import print_function

import argparse
import os
import subprocess
import sys
import webbrowser
import zipfile

from six.moves.SimpleHTTPServer import SimpleHTTPRequestHandler
from six.moves.socketserver import TCPServer


def str2bool(value):
    # From: https://stackoverflow.com/a/19233287/7829525
    return value.lower() in ("yes", "y", "true", "t", "1")


parser = argparse.ArgumentParser()
parser.register('type', 'bool', str2bool)
parser.add_argument(
    "--browser", type='bool', default=True, metavar='BOOL',
    help="Open browser. Disable this if you are frequently recompiling.")
args = parser.parse_args()

# Unpack zipfile and chdir into it.
with zipfile.ZipFile("doc/sphinx.zip", "r") as archive:
    archive.extractall("sphinx-tmp")
os.chdir("sphinx-tmp")
file_url = "file://%s/index.html " % os.path.abspath(os.getcwd())


# An HTTP handler without logging.
class Handler(SimpleHTTPRequestHandler):
    def log_request(*_):
        pass


# Serve the current directory for local browsing.
sockaddr = ("127.0.0.1", 8000)
TCPServer.allow_reuse_address = True
httpd = TCPServer(sockaddr, Handler)
http_url = "http://%s:%s/index.html" % sockaddr

# Users can click these as a backup, if the auto-open below doesn't work.
print("Sphinx preview docs are available at:", file=sys.stderr)
print("", file=sys.stderr)
print("  " + http_url, file=sys.stderr)
print("", file=sys.stderr)
print("  " + file_url, file=sys.stderr)
print("", file=sys.stderr)

# Try the default browser, then wait.
if args.browser:
    print("Opening webbrowser", file=sys.stderr)
    if sys.platform == "darwin":
        # macOS
        webbrowser.open(http_url)
    else:
        # Ubuntu
        webbrowser.open("./index.html")

print("Serving and waiting ... use Ctrl-C to exit.", file=sys.stderr)
httpd.serve_forever()
