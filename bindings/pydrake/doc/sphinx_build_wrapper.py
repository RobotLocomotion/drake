# -*- coding: UTF-8 -*-

import os
import sys


runfiles_dir = os.environ.get('DRAKE_BAZEL_RUNFILES')
assert runfiles_dir, 'Environment variable DRAKE_BAZEL_RUNFILES is NOT set.'
sphinx_build_path = os.path.join(runfiles_dir, 'external/sphinx/sphinx-build')
assert os.path.exists(sphinx_build_path), \
    'Path {} does NOT exist.'.format(sphinx_build_path)
os.environ['LANG'] = 'en_US.UTF-8'
argv = [sphinx_build_path] + sys.argv[1:]
os.execv(argv[0], argv)
