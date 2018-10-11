# -*- coding: UTF-8 -*-

import os
import sys

from sphinx.cmd.build import main

os.environ['LANG'] = 'en_US.UTF-8'
main(sys.argv[1:])
