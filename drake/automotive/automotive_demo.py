# Do not run this program directly; only use the compiled form in bazel-bin.

"""Launch the `automotive_demo` simulation with the following supporting
applications:

 * steering_command_driver.py for interactive input
 * drake-visualizer to see things move
 * lcm-spy to see LCM traffic of state and visualization
 * lcm-logger to capture LCM activity to disk

To kill all the processes, just kill the script in the console with
Control-C.
"""

import argparse
import fcntl
import glob
import os
import select
import subprocess
import sys
import time

import lcm

_epilog = """
All remaining arguments are passed to the %s program:
---
"""


class TrackedProcess(object):
    """A handy wrapper for a process object that remembers its label, and adds
    the fileno() accessor for select() compatibility.
    """
    def __init__(self, label, process):
        self.label = label
        self.process = process

    def fileno(self):
        return self.process.stdout.fileno()


class Launcher(object):
    """Launch and manage a group of processes as a group. It one exits, all
    the rest are killed. The aggregate return code is that of whichever one
    exited first.

    Any process output to stdout or stderr is echoed to the stdout of this
    script, prefixed by process-specific labels.
    """
    def __init__(self):
        self.dry_run = False
        self.children = []  # list of TrackedProcess
        self.devnull = open('/dev/null')
        self.returncode = None  # First one to exit wins.
        self.name = os.path.basename(__file__)

    def set_dry_run(self, dry_run_value):
        self.dry_run = dry_run_value

    def launch(self, command, label=None):
        """Launch a process to be managed with the group. If no label is
        supplied, a label is synthesized from the supplied command line.
        """
        if self.dry_run:
            print ' '.join(command)
            return

        if label is None:
            label = os.path.basename(command[0])
        if not os.path.exists(command[0]):
            print "[%s] Missing file %s; available files are:" % (
                self.name, command[0])
            sys.stdout.flush()
            subprocess.call(["/usr/bin/find", "-L", "."])
            raise RuntimeError(command[0] + " not found")
        process = subprocess.Popen(
            command,
            stdin=self.devnull,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
        fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.children.append(TrackedProcess(label, process))

        # Fail-fast on infant mortality.
        time.sleep(0.05)
        self._poll()
        if self.returncode is not None:
            print process.stdout.read(),
            print "[%s] %s failed to launch" % (self.name, label)
            sys.exit(self.returncode or 1)

    def _poll(self):
        for child in self.children:
            ret = child.process.poll()
            if ret is not None:
                print "[%s] %s exited %d" % (self.name, child.label, ret)
                if self.returncode is None:
                    self.returncode = ret
        return self.returncode

    def _wait(self, duration):
        done = False
        start = time.time()
        while not done:
            rlist, _, _ = select.select(self.children, [], [], 0.1)
            if self._poll() is not None:
                done = True

            now = time.time()
            elapsed = now - start
            if elapsed > duration:
                print "[%s] %s exited via duration elapsed" % (
                    self.name, self.name)
                self.returncode = 0
                done = True

            for child in rlist:
                try:
                    lines = child.process.stdout.read().splitlines()
                except IOError:
                    lines = []

                lines = [l for l in lines if l]
                if not lines:
                    continue

                print '\n'.join(
                    ["[%s] %s" % (child.label, line) for line in lines])

    def wait(self, duration):
        """Wait for any of the managed processes to exit, for a keyboard
        interrupt from the user, or for the specified duration to expire.
        Print a message explaining which event occurred. Set the return
        code as that of the first-exiting process, or 0 for keyboard
        interrupt or timeout.
        """
        if self.dry_run:
            return

        try:
            self._wait(duration)
        except KeyboardInterrupt:
            # This is considered success; we ran until the user stopped us.
            print "[%s] %s exited via keyboard interrupt" % (
                self.name, self.name)
            self.returncode = 0
        assert self.returncode is not None

    def kill(self):
        """Kill any still-running managed processes."""
        for child in self.children:
            if child.process.poll() is None:
                child.process.kill()

the_launcher = Launcher()


def wait_for_lcm_message_on_channel(channel):
    """Wait for a single message to arrive on the specified LCM channel.
    """
    m = lcm.LCM()

    def receive(channel, data):
        _ = (channel, data)
        raise StopIteration()

    sub = m.subscribe(channel, receive)
    start_time = time.time()
    try:
        while True:
            if time.time() - start_time > 10.:
                raise RuntimeError(
                    "Timeout waiting for channel %s" % channel)
            rlist, _, _ = select.select([m], [], [], 0.1)
            if m in rlist:
                m.handle()
    except StopIteration:
        pass
    finally:
        m.unsubscribe(sub)


def main():
    demo_path = "drake/automotive/automotive_demo"
    steering_command_driver_path = "drake/automotive/steering_command_driver"
    drake_visualizer_path = "external/drake_visualizer/drake-visualizer"
    lcm_spy_path = "drake/automotive/lcm-spy"
    lcm_logger_path = "external/lcm/lcm-logger"

    parser = argparse.ArgumentParser(
        add_help=False, description=__doc__, epilog=_epilog % demo_path,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=float('Inf'),
                        help="demo run duration in seconds")
    parser.add_argument("--steering-command-driver", action='store_true',
                        dest='launch_steering_command_driver', default=True,
                        help="launch steering_command_driver (default)")
    parser.add_argument("--no-steering-command-driver", action='store_false',
                        dest='launch_steering_command_driver', default=True,
                        help="don't launch steering_command_driver")
    parser.add_argument("--visualizer", action='store_true',
                        dest='launch_visualizer',
                        default=True, help="launch drake-visualizer (default)")
    parser.add_argument("--no-visualizer", action='store_false',
                        dest='launch_visualizer',
                        default=True, help="don't launch drake-visualizer")
    parser.add_argument("--dry-run", action='store_true',
                        default=False,
                        help="print commands instead of running them")
    args, tail = parser.parse_known_args()

    if '--help' in tail:
        parser.print_help(file=sys.stderr)
        subprocess.call([demo_path, "--help"])
        sys.exit(1)

    if args.dry_run:
        the_launcher.set_dry_run(True)

    try:
        print "*** LCM logs will be in", os.getcwd()
        the_launcher.launch([lcm_logger_path])
        the_launcher.launch([lcm_spy_path])

        if args.launch_visualizer:
            the_launcher.launch([drake_visualizer_path])
            # Await a message on the DRAKE_VIEWER_STATUS channel indicating
            # that drake-visualizer is ready. This ensures that the demo
            # app's LOAD_ROBOT message will be seen and processed.
            if args.dry_run:
                print(
                    "# wait_for_lcm_message_on_channel('DRAKE_VIEWER_STATUS')")
            else:
                wait_for_lcm_message_on_channel('DRAKE_VIEWER_STATUS')

        the_launcher.launch([demo_path] + tail)

        if args.launch_steering_command_driver:
            the_launcher.launch([steering_command_driver_path])

        the_launcher.wait(args.duration)

    finally:
        the_launcher.kill()

    sys.exit(the_launcher.returncode)

if __name__ == '__main__':
    main()
