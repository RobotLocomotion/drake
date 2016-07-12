#!/usr/bin/python

"""cpplint_wrapper.py -- Run cpplint.py using Drake's standard settings,
summarizing its output for cleanliness, and providing a --fast option
to run multiple linters in parallel.
"""

import argparse
import multiprocessing
import os
import subprocess
import sys


def summarize_cpplint(cmdline_and_files):
    """Given a cpplint subprocess command line (the program, arguments, and files
    to check), run cpplint and return a list of errors, or an empty list if
    there were no errors.
    """

    try:
        subprocess.check_output(
            cmdline_and_files,
            stderr=subprocess.STDOUT)
        sys.stdout.write('.')  # Progress bar.
        sys.stdout.flush()
        return []  # No errors.
    except subprocess.CalledProcessError as e:
        # Filter out known non-errors from everything else.
        errors = []
        for line in e.output.splitlines():
            if line.startswith('Done processing '):
                continue
            if line.startswith('Total errors '):
                continue
            errors.append(line)
        if not errors:
            # Our filtering failed, so report everything.
            errors = [e.output or "NO OUTPUT"]
        return errors

def multiprocess_cpplint(cmdline, files, num_processes):
    """Given a cpplint subprocess command line (just the program and arguments),
    separate list of files, and number of processess (None for "all CPUs"), run
    cpplint, display a progress bar, warning summary, and return a shell
    exitcode (0 on success, 1 on failure).
    """

    # Lint the files N at a time, to amortize interpreter start-up.
    N = 10
    files_groups = [files[i:i+N] for i in xrange(0, len(files), N)]
    cmdlines = [cmdline + some_files for some_files in files_groups]

    # Farm out each chunk to a process in a Pool.
    sys.stdout.write('Checking ...')  # Progress bar.
    sys.stdout.flush()
    pool = multiprocessing.Pool(processes=num_processes)
    errors = [
        error
        for errors in pool.map(summarize_cpplint, cmdlines)
        for error in errors]
    num_errors = len(errors)
    if num_errors == 0:
        print ' %d files passed' % len(files)  # Progress bar.
        return 0
    else:
        print ' failed with %d warnings' % num_errors  # Progress bar.
        for line in errors:
            print >>sys.stderr, line
        return 1

def main():
    # Find cpplint.py.
    drake_common_test_dir = os.path.dirname(os.path.abspath(__file__))
    drake_dir = os.path.abspath(os.path.join(drake_common_test_dir, '../../'))
    drake_distro_dir = os.path.abspath(os.path.join(drake_dir, '../'))
    default_cpplint = os.path.join(
        drake_distro_dir, 'externals/google_styleguide/cpplint/cpplint.py')

    # Prepare to parse our arguments.
    parser = argparse.ArgumentParser(
        description=__doc__.strip(),
        usage=
        '%(prog)s [options] [pathname [pathname ...]] [-- <cpplint.py args>]')
    parser.add_argument(
        '--cpplint',
        default=default_cpplint,
        help='path to cpplint.py (%(default)s)')
    parser.add_argument(
        '--extension', metavar='EXT', action='append', dest='extensions',
        default='c|cc|cpp|cxx|c++|h|hpp|hxx|h++'.split('|'),
        help='add an allowed extension (%(default)s)')
    parser.add_argument(
        '--no-summarize', action='store_false',
        dest='summarize', default='True',
        help='do not elide any cpplint output')
    parser.add_argument(
        '--fast', dest='num_processes', default=1,
        action='store_const', const=None,
        help='run in parallel with many threads')
    parser.add_argument(
        'pathnames', nargs='*', default=[drake_dir],
        help='list of files and/or directories to check'
        ' (default is all of Drake)')

    # Separate out extra_args, then parse the rest of the arguments.
    argv = sys.argv[1:]
    extra_args = []
    if '--' in argv:
        index = argv.index('--')
        extra_args = argv[index + 1:]
        argv[index:] = []
    args = parser.parse_args(argv)

    # Find every file under args.pathnames that matches args.extensions.
    files = [
        os.path.join(dirpath, filename)
        for pathname in args.pathnames
        for dirpath, _, filenames in os.walk(pathname)
        for filename in filenames
        if os.path.splitext(filename)[1][1:] in args.extensions]

    # Invoke cpplint.py.
    cmdline = [
        sys.executable, args.cpplint,
        "--extensions='%s'" % ','.join(args.extensions),
        '--output=eclipse',
    ] + extra_args
    if args.summarize:
        return multiprocess_cpplint(cmdline, files, args.num_processes)
    else:
        return subprocess.call(cmdline + files)

if __name__ == '__main__':
    sys.exit(main())
