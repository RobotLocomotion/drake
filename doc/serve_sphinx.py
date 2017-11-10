"""
This program serves sphinx.zip for a web browser.

Run this via:
  $ bazel run //doc:serve_sphinx
"""

import os
import subprocess
import sys
import webbrowser
import zipfile
from SimpleHTTPServer import SimpleHTTPRequestHandler
from SocketServer import TCPServer

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
print >>sys.stderr, "Sphinx preview docs are available at:"
print >>sys.stderr
print >>sys.stderr, "  " + http_url
print >>sys.stderr
print >>sys.stderr, "  " + file_url
print >>sys.stderr

# Try the default browser, then wait.
print >>sys.stderr, "Opening webbrowser and waiting ... use Ctrl-C to exit."
if sys.platform == "darwin":
    # macOS
    webbrowser.open(http_url)
else:
    # Ubuntu
    webbrowser.open("./index.html")
httpd.serve_forever()
