import platform
import os
import subprocess

from clang import cindex


# Alternative: Make this a function in `mkdoc.py`, and import it from mkdoc as
# a module? (if this was really authored in `mkdoc.py`...)


def add_library_paths():
    """Set library paths for finding libclang on supported platforms.

    Returns:
        parameters (list)
    """
    parameters = []
    library_file = None
    if platform.system() == 'Darwin':
        completed_process = subprocess.run(['xcrun', '--find', 'clang'],
                                           stdout=subprocess.PIPE,
                                           encoding='utf-8')
        if completed_process.returncode == 0:
            toolchain_dir = os.path.dirname(os.path.dirname(
                completed_process.stdout.strip()))
            library_file = os.path.join(
                toolchain_dir, 'lib', 'libclang.dylib')
        completed_process = subprocess.run(['xcrun', '--show-sdk-path'],
                                           stdout=subprocess.PIPE,
                                           encoding='utf-8')
        if parameters is not None and completed_process.returncode == 0:
            sdkroot = completed_process.stdout.strip()
            if os.path.exists(sdkroot):
                parameters += ['-isysroot', sdkroot]
    elif platform.system() == 'Linux':
        library_file = '/usr/lib/x86_64-linux-gnu/libclang-9.so'
    if not os.path.exists(library_file):
        raise RuntimeError(f'Library file {library_file} does NOT exist')
    cindex.Config.set_library_file(library_file)
    return parameters
