"""Tool to help with controlled benchmark experiments.

If necessary, installs software for CPU speed adjustment. Runs a bazel target,
with CPU speed control disabled. Copies result data to a user selected output
directory. Only supported on Ubuntu 20.04.

The purpose of CPU speed control for benchmarking is to disable automatic CPU
speed scaling, so that results of similar experiments will be more repeatable,
and comparable across experiments. Performance "in the wild" with scaling
enabled may be faster or slower, with higher variance.

This operation uses `sudo` commands to install tools for CPU scaling control
and to actually change the CPU configuration.
"""

import argparse
import contextlib
import os
import re
import shlex
import subprocess
import sys
import time


def is_default_ubuntu():
    """Return True iff platform is Ubuntu 20.04."""
    if os.uname().sysname != "Linux":
        return False
    release_info = subprocess.check_output(
                ["lsb_release", "-irs"], encoding='utf-8')
    return ("Ubuntu\n20.04" in release_info)


def get_installed_version(package_name):
    """Returns the installed version of a package, or None."""
    result = subprocess.run(
        ['dpkg-query', '--showformat=${db:Status-Abbrev} ${Version}',
         '--show', package_name],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        encoding='utf-8')
    if result.returncode != 0:
        return None
    status, version = result.stdout.split()
    if status != "ii":
        return None
    return version


def say(*args):
    """Print all the args, formatted for visibility."""
    print(f"\n=== {' '.join(args)} ===\n")


def sudo(*args, quiet=False):
    """Run sudo, passing all args to it."""
    new_args = ["sudo"] + list(args)
    print('Running: ', shlex.join(new_args))
    if quiet:
        popen = subprocess.Popen(
            new_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding='utf-8')
        if popen.wait() != 0:
            print(popen.stdout.read())
            raise RuntimeError("Failure during sudo()")
    else:
        subprocess.run(new_args, stderr=subprocess.STDOUT, check=True)


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
        sudo('cpupower', 'frequency-set', '--governor', governor, quiet=True)

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


def do_benchmark(args):
    if not CpuSpeedSettings().is_supported_cpu():
        raise RuntimeError("""
The intel_pstate Linux kernel driver is not running. Without it, there is no
way to prevent Turbo Boost cpu frequency scaling, and experiment results will
be invalid.
""")

    command_prologue = []
    if is_default_ubuntu():
        kernel_name = subprocess.check_output(
            ['uname', '-r'], encoding='utf-8').strip()
        kernel_packages = [f'linux-tools-{kernel_name}', 'linux-tools-common']
        if not all([get_installed_version(x) for x in kernel_packages]):
            say("Install tools for CPU speed control. [Note: sudo!]")
            sudo('apt', 'install', *kernel_packages)
        command_prologue = ["taskset", "--cpu-list", str(args.cputask)]

    if args.sleep:
        say(f"Wait {args.sleep} seconds for lingering activity to subside.")
        time.sleep(args.sleep)

    os.mkdir(args.output_dir)
    default_args = [
        '--benchmark_display_aggregates_only=true',
        '--benchmark_out_format=json',
        f'--benchmark_out={args.output_dir}/results.json',
    ]
    command = command_prologue + [args.binary] + default_args + args.extra_args
    with open(f'{args.output_dir}/summary.txt', 'wb') as summary:
        with CpuSpeedSettings().scope(governor="performance", no_turbo="1"):
            say("Run the experiment.")
            print('Running: ', shlex.join(command))
            popen = subprocess.Popen(command, stdout=subprocess.PIPE)
            for line in popen.stdout:
                summary.write(line)
                print(line.decode("utf-8").strip(), flush=True)
            if popen.wait() != 0:
                raise RuntimeError("The profiled BINARY has failed")


def main():
    # Make cwd be what the user expected, not the runfiles tree.
    assert ".runfiles" in ':'.join(sys.path), "Always use 'bazel run'."
    os.chdir(os.environ['BUILD_WORKING_DIRECTORY'])

    # Parse and validate arguments.
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        '--binary', metavar='BINARY', required=True,
        help='path to googlebench binary; typically this is supplied'
             ' automatically by the drake_py_experiment_binary macro')
    parser.add_argument(
        '--output_dir', metavar='OUTPUT-DIR', required=True,
        help='output directory for benchmark data; it must not already exist')
    parser.add_argument(
        '--sleep', type=float, default=10.0,
        help='pause this long for lingering activity to subside (in seconds)')
    parser.add_argument(
        # Defaulting to processor #0 is arbitrary; it is up to experimenters to
        # ensure it is idle during experiments or else specify a different one.
        '--cputask', type=int, metavar='N', default=0,
        help='pin the BINARY to vcpu number N for this experiment')
    parser.add_argument(
        'extra_args', nargs='*',
        help='extra arguments passed to the underlying executable')
    args = parser.parse_args()
    if not os.path.exists(args.binary):
        parser.error("BINARY does not exist .")
    if os.path.exists(args.output_dir):
        parser.error("OUTPUT-DIR must not already exist.")

    # Run.
    do_benchmark(args)


if __name__ == '__main__':
    main()
