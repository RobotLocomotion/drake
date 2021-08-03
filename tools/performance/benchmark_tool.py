"""Tool to help with controlled benchmark experiments.

Subcommands:

conduct_experiment:
  If necessary, install software for CPU speed adjustment. Run a bazel target,
  with CPU speed control disabled. Copy result data to a user selected output
  directory. Only supported on Ubuntu 18.04.

  The purpose of CPU speed control for benchmarking is to disable automatic CPU
  speed scaling, so that results of similar experiments will be more
  repeatable, and comparable across experiments. Performance "in the wild" with
  scaling enabled may be faster or slower, with higher variance.

  This operation uses `sudo` commands to install tools for CPU scaling control
  and to actually change the CPU configuration.

  It is possible to customize the bazel build and run steps by using
  "--extra-build-args=" (a.k.a. "-b=") options:

    python3 -B benchmark_tool.py conduct_experiment \\
      -b=--copt=-mtune=haswell \\
      -b=--copt=-g \\
      [TARGET] [OUTPUT-DIR]

copy_results:
  Copy results of a prior run of a designated target to a user selected output
  directory.
"""

import argparse
import contextlib
import os
import re
import shlex
import shutil
import subprocess
import sys
import time

WORKSPACE = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))


def is_default_ubuntu():
    """Return True iff platform is Ubuntu 18.04."""
    if os.uname().sysname != "Linux":
        return False
    release_info = subprocess.check_output(
                ["lsb_release", "-irs"], encoding='utf-8')
    return ("Ubuntu\n18.04" in release_info)


def say(*args):
    """Print all the args, formatted for visibility."""
    print(f"\n=== {' '.join(args)} ===\n")


def run(*args, echo=False):
    """Run a subprocess, checking for failure. Optionally echo the command."""
    if echo:
        print('Running: ', ' '.join([shlex.quote(x) for x in args]))
    subprocess.run(list(args), check=True)


def sudo(*args):
    """Run sudo, passing all args to it."""
    run('sudo', *args, echo=True)


class CpuSpeedSettings:
    """Routines for controlling CPU speed."""

    # This is the Linux kernel configuration file for Intel's "turbo boost".
    # https://www.kernel.org/doc/html/v4.12/admin-guide/pm/intel_pstate.html#no-turbo-attr
    NO_TURBO_CONTROL_FILE = "/sys/devices/system/cpu/intel_pstate/no_turbo"

    def is_supported_cpu(self):
        """Returns True if the current CPU is supported for speed control."""
        return os.path.exists(self.NO_TURBO_CONTROL_FILE)

    def get_cpu_governor(self):
        """Return the current CPU governor name string."""
        text = subprocess.check_output(
            ["cpupower", "frequency-info", "-p"], encoding='utf-8')
        m = re.search(r'\bgovernor "([^"]*)" ', text)
        return m.group(1)

    def set_cpu_governor(self, governor):
        """Set the CPU governor to the given name string."""
        sudo('cpupower', 'frequency-set', '--governor', governor)

    def get_no_turbo(self):
        """Return the current no-turbo state as string, either '1' or '0'."""
        with open(self.NO_TURBO_CONTROL_FILE, 'r', encoding='utf-8') as fo:
            return fo.read().strip()

    def set_no_turbo(self, no_turbo):
        """Set the no-turbo state to the given no_turbo string."""
        sudo('sh', '-c', f"echo {no_turbo} > {self.NO_TURBO_CONTROL_FILE}")

    @contextlib.contextmanager
    def scope(self, governor, no_turbo):
        """Context manager that sets governor and no_turbo states and
        restores the old state afterward.
        """
        say("Control CPU speed variation. [Note: sudo!]")
        old_gov = self.get_cpu_governor()
        old_nt = self.get_no_turbo()
        try:
            self.set_cpu_governor(governor)
            self.set_no_turbo(no_turbo)
            yield
        finally:
            say("Restore CPU speed settings. [Note: sudo!]")
            self.set_no_turbo(old_nt)
            self.set_cpu_governor(old_gov)


def target_to_path_fragment(target):
    """Convert a bazel target to a path fragment."""
    target_fragment = target
    if target_fragment.startswith('//'):
        target_fragment = target_fragment[2:]
    return os.path.join(*target_fragment.split(':'))


def conduct_experiment(args):
    """Implement the conduct_experiment sub-command."""
    say("Validate environment.")
    assert is_default_ubuntu(), (
        "experiments only supported on default platform")
    assert CpuSpeedSettings().is_supported_cpu(), (
        "experiments only supported with Intel CPUs")

    say("Install tools for CPU speed control. [Note: sudo!]")
    kernel_name = subprocess.check_output(
        ['uname', '-r'], encoding='utf-8').strip()
    sudo('apt', 'install', f'linux-tools-{kernel_name}', 'linux-tools-common')

    say("Build code.")
    run('bazel', 'build', *args.extra_build_args, args.target)

    sleep_seconds = 10
    say(f"Wait {sleep_seconds} seconds for lingering activity to subside.")
    time.sleep(sleep_seconds)

    with CpuSpeedSettings().scope(governor="performance", no_turbo="1"):
        say("Run the experiment.")
        run('bazel', 'run',
            *args.extra_build_args, args.target, '--', *args.extra_args)

    say(f"Save data to {args.output_dir}/.")
    copy_results(args)


def copy_results(args):
    """Implement the copy_results sub-command."""
    src = os.path.join(WORKSPACE, 'bazel-testlogs',
                       target_to_path_fragment(args.target),
                       'test.outputs')
    shutil.copytree(src, args.output_dir)


def main():
    # Don't run under bazel; this program issues bazel commands.
    assert "runfiles" not in ':'.join(sys.path), "Don't run under bazel!"

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(
        help='subcommand to run', dest='subcommand')
    subparsers.required = True

    def add_common_args(parser):
        parser.add_argument(
            'target', metavar='TARGET',
            help='bazel target of benchmark program')
        parser.add_argument(
            'output_dir', metavar='OUTPUT-DIR',
            help='output directory for benchmark data;'
            ' it must not already exist.')

    parser_conduct_experiment = subparsers.add_parser(
        'conduct_experiment',
        description='run controlled experiment for TARGET,'
        ' copying benchmark data to OUTPUT-DIR')
    add_common_args(parser_conduct_experiment)
    parser_conduct_experiment.add_argument(
        '-b', '--extra-build-args', action='append', default=[],
        help='extra arguments passed to bazel build and run')
    parser_conduct_experiment.add_argument(
        'extra_args', nargs='*',
        help='extra arguments passed to the underlying executable')
    parser_conduct_experiment.set_defaults(func=conduct_experiment)

    parser_copy_results = subparsers.add_parser(
        'copy_results',
        description='copy benchmark data for TARGET'
        ' from the bazel-testlogs tree to OUTPUT-DIR')
    add_common_args(parser_copy_results)
    parser_copy_results.set_defaults(func=copy_results)

    args = parser.parse_args()

    # Fail fast if the output directory is an empty string or already exists.
    assert args.output_dir, \
        "OUTPUT-DIR must not be 0-length."
    assert not os.path.exists(args.output_dir), \
        "OUTPUT-DIR must not already exist."

    args.func(args)


if __name__ == '__main__':
    main()
