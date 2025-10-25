"""Tool to help with controlled benchmark experiments.

If necessary, installs software for CPU speed adjustment. Runs a bazel target,
with CPU speed control disabled. Copies result data to a user selected output
directory. Only supported on Ubuntu 24.04.

The purpose of CPU speed control for benchmarking is to disable automatic CPU
speed scaling, so that results of similar experiments will be more repeatable,
and comparable across experiments. Performance "in the wild" with scaling
enabled may be faster or slower, with higher variance.

This operation uses `sudo` commands to install tools for CPU scaling control
and to actually change the CPU configuration.
"""

import argparse
import contextlib
import copy
import os
import re
import shlex
import subprocess
import sys
import time


def say(*args):
    """Print all the args, formatted for visibility."""
    print(f"\n=== {' '.join(args)} ===\n")


def sudo(*args, quiet=False):
    """Run sudo, passing all args to it."""
    new_args = ["sudo"] + list(args)
    print("Running: ", shlex.join(new_args))
    if quiet:
        popen = subprocess.Popen(
            new_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            encoding="utf-8",
        )
        if popen.wait() != 0:
            print(popen.stdout.read())
            raise RuntimeError("Failure during sudo()")
    else:
        subprocess.run(new_args, stderr=subprocess.STDOUT, check=True)


class NoBoost:
    def is_supported(self):
        return True

    def get_boost(self):
        """Return the current boost state; True means boost is enabled."""
        return False

    def set_boost(self, boost_value):
        """Set the boost state; True means boost is enabled."""
        say(
            "No method of cpu boost control was found;"
            " nothing is changed. Benchmark results may be noisy."
        )


class IntelBoost:
    # This is the Linux kernel configuration file for Intel's "turbo boost".
    # https://www.kernel.org/doc/html/v4.12/admin-guide/pm/intel_pstate.html#no-turbo-attr
    NO_TURBO_CONTROL_FILE = "/sys/devices/system/cpu/intel_pstate/no_turbo"

    def is_supported(self):
        return os.path.exists(self.NO_TURBO_CONTROL_FILE)

    def get_boost(self):
        """Return the current boost state; True means boost is enabled."""
        with open(self.NO_TURBO_CONTROL_FILE, "r", encoding="utf-8") as fo:
            no_turbo = int(fo.read().strip())
            return not no_turbo  # Intel reverses the sense.

    def set_boost(self, boost_value):
        """Set the boost state; True means boost is enabled."""
        no_turbo = int(not boost_value)  # Intel reverses the sense.
        sudo("sh", "-c", f"echo {no_turbo} > {self.NO_TURBO_CONTROL_FILE}")


class LinuxKernelBoost:
    # This is the Linux kernel configuration file for chip-agnostic boost
    # control.
    # https://www.kernel.org/doc/html/v5.19/admin-guide/pm/cpufreq.html#frequency-boost-support
    CPUFREQ_BOOST_FILE = "/sys/devices/system/cpu/cpufreq/boost"

    def is_supported(self):
        return os.path.exists(self.CPUFREQ_BOOST_FILE)

    def get_boost(self):
        """Return the current boost state; True means boost is enabled."""
        with open(self.CPUFREQ_BOOST_FILE, "r", encoding="utf-8") as fo:
            return bool(fo.read().strip())

    def set_boost(self, boost_value):
        """Set the boost state; True means boost is enabled."""
        sudo("sh", "-c", f"echo {int(boost_value)} > {self.CPUFREQ_BOOST_FILE}")


class CpuSpeedSettings:
    """Routines for controlling CPU speed."""

    def __init__(self, *, cpu: int):
        """The `cpu` specifies which specific cpu to govern. We disable boost
        for the entire machine, but we only govern the frequency of this one
        specific cpu."""
        assert isinstance(cpu, int)
        self._cpu = str(cpu)
        self._boost = None
        for boost in [LinuxKernelBoost, IntelBoost, NoBoost]:
            if boost().is_supported():
                self._boost = boost()
                break
        self._ensure_cpupower_exists()

    @staticmethod
    def _ensure_cpupower_exists():
        if os.path.exists("/usr/bin/cpupower"):
            return
        say("Install tools for CPU speed control. [Note: sudo!]")
        kernel_name = subprocess.check_output(
            ["uname", "-r"], encoding="utf-8"
        ).strip()
        kernel_packages = [f"linux-tools-{kernel_name}", "linux-tools-common"]
        sudo("apt", "install", *kernel_packages)

    def is_supported_cpu(self):
        """Returns True if the current CPU is supported for speed control."""
        return not isinstance(self._boost, NoBoost)

    def get_cpu_governor(self):
        """Return the current CPU governor name string."""
        text = subprocess.check_output(
            ["cpupower", "-c", self._cpu, "frequency-info", "-p"],
            encoding="utf-8",
        )
        m = re.search(r'\bgovernor "([^"]*)" ', text)
        return m.group(1)

    def set_cpu_governor(self, governor):
        """Set the CPU governor to the given name string."""
        sudo(
            "cpupower",
            "-c",
            self._cpu,
            "frequency-set",
            "--governor",
            governor,
            quiet=True,
        )

    def get_boost(self):
        """Return the current boost state; True means boost is enabled."""
        return self._boost.get_boost()

    def set_boost(self, boost_value):
        """Set the boost state; True means boost is enabled."""
        return self._boost.set_boost(boost_value)

    @contextlib.contextmanager
    def scope(self, governor, boost):
        """Context manager that sets governor and boost states and
        restores the old state afterward.
        """
        say("Control CPU speed variation. [Note: sudo!]")
        old_gov = self.get_cpu_governor()
        old_boost = self.get_boost()
        try:
            self.set_cpu_governor(governor)
            self.set_boost(boost)
            yield
        finally:
            say("Restore CPU speed settings. [Note: sudo!]")
            self.set_boost(old_boost)
            self.set_cpu_governor(old_gov)


def do_benchmark(args):
    cpu_speed_settings = CpuSpeedSettings(cpu=args.cputask)
    if not cpu_speed_settings.is_supported_cpu():
        say(f"""
No method of controlling cpu frequency scaling was detected. Without it, there
is no way to prevent arbitrary cpu frequency scaling, and experiment results
will be invalid. Supported methods are:

 * (newer) Linux kernels, controlled through
   {LinuxKernelBoost().CPUFREQ_BOOST_FILE}.
 * intel_pstate driver, controlled through
   {IntelBoost().NO_TURBO_CONTROL_FILE}.
""")

    if args.sleep:
        say(f"Wait {args.sleep} seconds for lingering activity to subside.")
        time.sleep(args.sleep)

    os.mkdir(args.output_dir)
    default_args = [
        "--benchmark_display_aggregates_only=true",
        "--benchmark_out_format=json",
        f"--benchmark_out={args.output_dir}/results.json",
    ]
    command = ["taskset", "--cpu-list", str(args.cputask)]
    command += [args.binary] + default_args + args.extra_args
    env = copy.copy(os.environ)
    env["DRAKE_GOOGLEBENCH_SUPPRESS_SCALING_WARNING"] = "1"
    with open(f"{args.output_dir}/summary.txt", "wb") as summary:
        with cpu_speed_settings.scope(governor="performance", boost=False):
            say("Run the experiment.")
            print("Running: ", shlex.join(command))
            popen = subprocess.Popen(command, stdout=subprocess.PIPE, env=env)
            for line in popen.stdout:
                summary.write(line)
                print(line.decode("utf-8").strip(), flush=True)
            if popen.wait() != 0:
                raise RuntimeError("The profiled BINARY has failed")


def main():
    # Make cwd be what the user expected, not the runfiles tree.
    assert ".runfiles" in ":".join(sys.path), "Always use 'bazel run'."
    os.chdir(os.environ["BUILD_WORKING_DIRECTORY"])

    # Parse and validate arguments.
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--binary",
        metavar="BINARY",
        required=True,
        help="path to googlebench binary; typically this is supplied"
        " automatically by the drake_py_experiment_binary macro",
    )
    parser.add_argument(
        "--output_dir",
        metavar="OUTPUT-DIR",
        required=True,
        help="output directory for benchmark data; it must not already exist",
    )
    parser.add_argument(
        "--sleep",
        type=float,
        default=10.0,
        help="pause this long for lingering activity to subside (in seconds)",
    )
    parser.add_argument(
        # Defaulting to processor #0 is arbitrary; it is up to experimenters to
        # ensure it is idle during experiments or else specify a different one.
        "--cputask",
        type=int,
        metavar="N",
        default=0,
        help="pin the BINARY to vcpu number N for this experiment",
    )
    parser.add_argument(
        "extra_args",
        nargs="*",
        help="extra arguments passed to the underlying executable",
    )
    args = parser.parse_args()
    if not os.path.exists(args.binary):
        parser.error("BINARY does not exist .")
    if os.path.exists(args.output_dir):
        parser.error("OUTPUT-DIR must not already exist.")

    # Get the password check out of the way before doing anything else.
    sudo("-v")

    # Run.
    do_benchmark(args)


if __name__ == "__main__":
    main()
