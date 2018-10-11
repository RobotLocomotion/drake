# -*- coding: UTF-8 -*-

import os
import sys
# N.B. Future versions use the setup:
#  from sphinx.cmd.build import main
#  sys.exit(main(sys.argv[1:]))
from sphinx import main

assert __name__ == "__main__"
os.environ['LANG'] = 'en_US.UTF-8'
sys.exit(main(sys.argv))
