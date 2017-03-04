"""
This program serves sphinx.zip for a web browser.

Run this via:
  $ bazel run //drake/doc:serve_sphinx
"""

import os
import subprocess
import sys
import zipfile
from SimpleHTTPServer import SimpleHTTPRequestHandler
from SocketServer import TCPServer

# Unpack zipfile and chdir into it.
with zipfile.ZipFile("drake/doc/sphinx.zip", "r") as archive:
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

# Try the default browser, then wait.
print >>sys.stderr, "Serving sphix docs ... use Ctrl-C to exit."
try:
    subprocess.check_call(["sensible-browser", file_url])
except:
    print >>sys.stderr, "Could not find web browser; use one of these links:"
    print >>sys.stderr
    print >>sys.stderr, "  " + http_url
    print >>sys.stderr
    print >>sys.stderr, "  " + file_url
    print >>sys.stderr

httpd.serve_forever()
