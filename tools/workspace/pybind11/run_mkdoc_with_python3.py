import os
import sys


def resolve_path(relpath):
    abspath = os.path.join(runfiles_dir, relpath)
    assert os.path.exists(abspath), 'Path {} does NOT exist.'.format(abspath)
    return abspath


def prepend_path(key, path):
    os.environ[key] = path + ':' + os.environ.get(key, '')


runfiles_dir = os.environ.get('DRAKE_BAZEL_RUNFILES')
assert runfiles_dir, 'Environment variable DRAKE_BAZEL_RUNFILES is NOT set.'

if sys.platform.startswith('linux'):
    prepend_path('LD_LIBRARY_PATH', '/usr/lib/llvm-4.0/lib')
    prepend_path('LD_LIBRARY_PATH', '/usr/lib/llvm-6.0/lib')

os.environ['LANG'] = 'en_US.UTF-8'

mkdoc_path = resolve_path('external/pybind11/tools/mkdoc.py')
python3_path = resolve_path('external/python3/python3')
argv = [python3_path, mkdoc_path] + sys.argv[1:]
os.execv(argv[0], argv)
