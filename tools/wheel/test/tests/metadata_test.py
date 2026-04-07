#!/usr/bin/env python3

import importlib.metadata

version = importlib.metadata.version("drake")
assert isinstance(version, str), repr(version)
