'''Command-line tool to compare one operation under two different git
commits.

This cannot serve as the basis of a regression test because the measured
behavior is strongly dependent on the machine and operating system on which the
test is run. It is intended to be a convenience utility to track local changes
that may be well correlated with global code improvements.

The purpose of this is to compare the runtime cost of a bazel runnable target
across changes in different commits. In principle, it is enough to specify
the bazel target and the commits to compapre.

However, it may be necessary to include command-line arguments to the bazel
run command. In this case, there are two kinds of command parameters:

  global: these are command-line parameters that apply to the bazel run
          target in every commit.
  per-commit: these are command-line parameters that apply only to the 
          associated commit (reflecting differences in API).

This utility uses the '--globals' and '--commit' flags to distinguish the two
types of parameters. As such, attempting to run a bazel target with flags that
look like either '--commit' or '--globals' will cause a parsing error. Avoid
using those parameters isolated in that way.

For example:

   python sha_compare.py //examples:foo --commit master --commit my_branch
      --disable_thing=1 --globals --run_all


Will do the following:

  - git checkout master
  - bazel build //examples:foo
  - For a number of iterations
    - ./bazel-bin/examples/foo --run_all
  - git checkout my_branch
  - bazel build //examples:foo
  - For a number of iterations
    - ./bazel-bin/examples/foo --run_all --disable_thing=1
  - Report the run time of the run operations.
'''

import argparse
import os
import subprocess
import sys
import time


def find_drake_root():
    '''Find the root of the drake directory from the current working
     directory'''
    # We're going to find the `bazel-bin` directory.
    def is_drake(path):
        return 'bazel-bin' in os.listdir(path) and \
            os.path.isdir(os.path.join(path, 'bazel-bin'))
    cwd = os.getcwd()
    while cwd:
        if is_drake(cwd):
            return cwd
        cwd = os.path.split(cwd)[0]
    raise ValueError("Somehow the current directory is not in the drake tree")


def get_executable_from_target(target):
    '''If we presume that `bazel build target` is valid for the current working
    directory, we want to construct a path from cwd to the built executable'''
    drake_root = find_drake_root()
    
    if (target[0] == ':'):
        return os.path.join(drake_root, 'bazel-bin', 
                            os.path.relpath(os.getcwd(), drake_root), 
                            target[1:])
    elif (target[:2] == '//'):
        return os.path.join(drake_root, 'bazel-bin',
                            target[2:].replace(':', '/'))
    else:
        return os.path.join(drake_root, 'bazel-bin', 
                            os.path.relpath(os.getcwd(), drake_root), 
                            target.replace(':', '/'))


def run_instance(target, sha, sha_params, global_params, iterations=1):
    '''Run the bazel target in the given git sha with the given parameters.
    Returns the user time reported by the linux command `timer`.'''
    print("Checking out branch '{}'".format(sha))
    result = subprocess.run(['git', 'checkout', sha])
    if result.returncode != 0:
        raise ValueError("Failed to check out commit: '{}'".format(sha))
    result = subprocess.run(['bazel', 'build', target])
    if result.returncode != 0:
        raise ValueError("Failed to build target: '{}'".format(target))
    
    prog = get_executable_from_target(target)
    args = [prog] + sha_params + global_params

    elapsed = []
    for i in range(iterations):
        start = time.time()
        result = subprocess.run(args)
        elapsed.append(time.time() - start)
        if result.returncode != 0:
            raise ValueError("Failed to run target: '{}'".format(target))
    return elapsed


def print_usage_and_exit(error_message = None):
    '''Prints usage information with an optional error message and exits the
    program.'''
    return_code = 0
    s = ''
    if error_message:
        s += '\nERROR: ' + error_message + '\n\n'
        return_code = 1
    s += 'usage: sha_compare.py [-h] TARGET\n'
    s += '                      [--commit commit [run_parameters ...]] ... \n'
    s += '                      [--globals global_param [global_param ...]]\n'
    s += '\n'
    s += __doc__.strip() + '\n'
    s += '\n\npositional arguments:\n'
    s += '  TARGET         The bazel target to build and run\n'
    s += '\n'
    s += 'Commit arguments:\n'
    s += '\n'
    s += '--commit COMMIT [run_parameter_0 ... ]   A commit label that can\n'
    s += '         bechecked out followed by any command-line parameters\n'
    s += '         that should be applied uniquely to the target under this\n'
    s += '         commit.\n'
    s += '--globals global_param ...   Command-line parameters that should\n'
    s += '                      be applied to the invocation of target for\n'
    s += '                      every commit\n'

    print(s)

    sys.exit(return_code)


class Namespace:
    '''Dummy class that mirrors tha argparse.Namespace such that parsed
    parameters end up as members of the parsed namespace.'''
    pass


def parse_args():
    '''Parses the command-line arguments. We can't use argparse because we want
    to do something it simply can't support. See file documentation above.'''
    if '-h' in sys.argv or '--help' in sys.argv:
        print_usage_and_exit()

    if len(sys.argv) < 2:
        print_usage_and_exit("Insufficient parameters - no target given")

    if len(sys.argv) < 4:
        print_usage_and_exit(
            "Insufficient parameters - likely no valid commit given")

    args = Namespace()
    arg_count = len(sys.argv)

    args.target = sys.argv[1]
    commits = []
    args.commits = commits
    args.globals = []
    args.iterations = 10
    i = 2
    curr_parameters = None
    while i < arg_count:
        p = sys.argv[i]
        if p == '--commit':
            i += 1
            if i >= arg_count:
                print_usage_and_exit(
                    "Commit tag '--commit' not followed by commit label")
            curr_parameters = []
            commits.append((sys.argv[i], curr_parameters))
        elif p == '--globals':
            curr_parameters = args.globals
        elif p == '--iterations':
            i += 1
            if i >= arg_count:
                print_usage_and_exit(
                    "Commit tag '--iterations' not followed by integer")
            try:
                args.iterations = int(sys.argv[i])
                curr_parameters = None
            except ValueError:
                print_usage_and_exit(
                    "Unable to convert '{}' to an int".format(p))
        else:
            if curr_parameters is None:
                print_usage_and_exit("Unrecognized parameter '{}'".format(p))
            else:
                curr_parameters.append(p)
        i += 1
    return args


def main():
    args = parse_args()

    sha_times = {}
    for sha, sha_params in args.commits:
        elapsed = run_instance(args.target, sha, sha_params, args.globals,
                               args.iterations)
        sha_times[sha] = elapsed

    max_length = max(len(sha) for sha in sha_times.keys()) + 2
    row_format = '{{:>{}}}  {{:15}} {{}}'.format(max_length)
    print(row_format.format('Commit', 'Average time (s)', 'Iterations'))
    for sha, times in sha_times.items():
        print(row_format.format(sha, '{:.5g}'.format(sum(times) / len(times)),
                                len(times)))
    

if __name__ == '__main__':
    main()
