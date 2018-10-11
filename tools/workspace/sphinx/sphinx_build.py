# -*- coding: UTF-8 -*-

import os
import sys
from sphinx.cmd.build import main

assert __name__ == "__main__"
os.environ['LANG'] = 'en_US.UTF-8'
sys.exit(main(sys.argv[1:]))
