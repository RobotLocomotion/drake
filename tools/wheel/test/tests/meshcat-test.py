#!/usr/bin/env python3

import urllib.request

from pydrake.geometry import Meshcat

meshcat = Meshcat()
with urllib.request.urlopen(meshcat.web_url()) as response:
    size = len(response.read())
assert size > 1000
