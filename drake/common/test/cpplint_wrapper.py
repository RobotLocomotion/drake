#!/usr/bin/python

"""cpplint_wrapper.py -- Run cpplint.py using Drake's standard settings,
summarizing its output for cleanliness, and providing a --fast option
to run multiple linters in parallel.
"""

import argparse
import functools
import multiprocessing
import os
import subprocess
import sys


def summarize_cpplint(cmdline_and_files, args):
    """Given a cpplint subprocess command line (the program, arguments, and
    files to check), run cpplint and return a list of errors, or an empty list
    if there were no errors.
    """

    try:
        output = subprocess.check_output(
            cmdline_and_files,
            stderr=subprocess.STDOUT)
        passed = True
    except subprocess.CalledProcessError as e:
        output = e.output
        passed = False

    # Filter out known non-errors from everything else.
    errors = []
    for line in output.splitlines():
        if line.startswith('Done processing '):
            if args.summarize:  # Progress bar.
                sys.stdout.write('.')
                sys.stdout.flush()
            else:
                sys.stdout.write(line + '\n')
                sys.stdout.flush()
            continue
        if line.startswith('Total errors '):
            continue
        if not passed:
            errors.append(line)
    if not passed and not errors:
        # Our filtering failed, so report everything.
        errors = [e.output or "NO OUTPUT"]
    return errors


def worker_summarize_cpplint(cmdline_and_files, args):
    """Escalate keyboard interrupts (Ctrl-C) in worker processes up to the
    master process.  There are still Ctrl-C race conditions where the pool
    keeps going, but this substantially reduces the window.
    """

    try:
        return summarize_cpplint(cmdline_and_files, args)
    except KeyboardInterrupt:
        raise Exception


def multiprocess_cpplint(cmdline, files, args):
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
    if args.summarize:  # Progress bar.
        sys.stdout.write('Checking ...')
        sys.stdout.flush()
    pool = multiprocessing.Pool(processes=args.num_processes)
    function = functools.partial(worker_summarize_cpplint, args=args)
    errors = [
        error
        for errors in pool.map(function, cmdlines)
        for error in errors]
    if args.summarize:  # Progress bar.
        sys.stdout.write('\n')
        sys.stdout.flush()

    # Act on the results.
    num_errors = len(errors)
    if num_errors == 0:
        print ' TOTAL %d files passed' % len(files)
        return 0
    else:
        print ' TOTAL %d files checked, found %d warnings' % (
            len(files), num_errors)
        for line in errors:
            print >>sys.stdout, line
        return 1


def main():
    # Find cpplint.py.
    drake_common_test_dir = os.path.dirname(os.path.abspath(__file__))
    drake_dir = os.path.abspath(os.path.join(drake_common_test_dir, '../..'))
    drake_distro_dir = os.path.abspath(os.path.join(drake_dir, '..'))
    drake_ros_dir = os.path.join(drake_distro_dir, 'ros')
    default_cpplint = os.path.join(
        drake_distro_dir, 'externals/google_styleguide/cpplint/cpplint.py')

    # Prepare to parse our arguments.
    parser = argparse.ArgumentParser(
        description=__doc__.strip(),
        usage=('%(prog)s [options] [pathname [pathname ...]]'
               ' [-- <cpplint.py args>]'))
    parser.add_argument(
        '--cpplint',
        default=default_cpplint,
        help='path to cpplint.py (default %(default)s)')
    parser.add_argument(
        '--extension', metavar='EXT', action='append', dest='extensions',
        default='c|cc|cpp|cxx|c++|h|hpp|hxx|h++'.split('|'),
        help='add an allowed extension (default %(default)s)')
    parser.add_argument(
        '--no-summarize', action='store_false',
        dest='summarize', default='True',
        help='show progress with filenames, instead of dots')
    parser.add_argument(
        '--num-processes', metavar='N', type=int, default=None,
        help='limit to this number of processes (default all CPUs)')
    parser.add_argument(
        'pathnames', nargs='*', default=[drake_dir, drake_ros_dir],
        help='list of files and/or directories to check'
        ' (default %(default)s)')

    # Separate out extra_args, then parse the rest of the arguments.
    argv = sys.argv[1:]
    extra_args = []
    if '--' in argv:
        index = argv.index('--')
        extra_args = argv[index + 1:]
        argv[index:] = []
    args = parser.parse_args(argv)

    # Search our args.pathnames.  For any directories it lists, recursively
    # find every file within that matches args.extensions.  For any files
    # it lists, if their extension matches then lint them; otherwise, show
    # a warning and do not count them in the total files passed.
    args_files = [x for x in args.pathnames if os.path.isfile(x)]
    args_nonfiles = [x for x in args.pathnames if not os.path.isfile(x)]
    files = [
        os.path.join(dirpath, filename)
        for pathname in args_nonfiles
        for dirpath, _, filenames in os.walk(pathname)
        for filename in filenames
        if os.path.splitext(filename)[1][1:] in args.extensions]
    for filename in args_files:
        if os.path.splitext(filename)[1][1:] not in args.extensions:
            print "Ignoring %s; not a valid file name." % filename
        else:
            files.append(filename)

    # Invoke cpplint.py.
    cmdline = [
        sys.executable, args.cpplint,
        "--extensions='%s'" % ','.join(args.extensions),
        '--output=eclipse',
    ] + extra_args
    return multiprocess_cpplint(cmdline, files, args)

if __name__ == '__main__':
    sys.exit(main())
