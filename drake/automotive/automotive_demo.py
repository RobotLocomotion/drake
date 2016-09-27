#!/usr/bin/env python

"""Launch the `automotive_demo` simulation with the following supporting
applications:

 * steering_command_driver.py for interactive input
 * drake-visualizer to see things move
 * bot-spy to see LCM traffic of state and visualization
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

from drake_paths import (add_module_search_paths, DRAKE_DIST_BUILD_DIR,
                         DRAKE_INSTALL_BIN_DIR, DRAKE_DRAKE_BIN_DIR)

add_module_search_paths()  # so we can find lcm stuff.

import lcm

_THIS_FILE = os.path.abspath(__file__)
_THIS_DIR = os.path.dirname(_THIS_FILE)

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
        self.children = []  # list of TrackedProcess
        self.devnull = open('/dev/null')
        self.returncode = None  # First one to exit wins.
        self.name = os.path.basename(_THIS_FILE)

    def launch(self, command, label=None):
        """Launch a process to be managed with the group. If no label is
        supplied, a label is synthesized from the supplied command line.
        """
        if label is None:
            label = os.path.basename(command[0])
        process = subprocess.Popen(
            command,
            stdin=self.devnull,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        flags = fcntl.fcntl(process.stdout, fcntl.F_GETFL)
        fcntl.fcntl(process.stdout, fcntl.F_SETFL, flags | os.O_NONBLOCK)
        self.children.append(TrackedProcess(label, process))

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


def bot_spy_that_actually_works():
    # Workaround for #3231.
    jar_dir = os.path.join(DRAKE_DIST_BUILD_DIR, "install", "share", "java")
    jar_glob = os.path.join(jar_dir, "*.jar")
    classpath = ':'.join(glob.glob(jar_glob) +
                         [os.environ.get("CLASSPATH", "")])
    the_launcher.launch(["java", "-cp", classpath, "lcm.spy.Spy"],
                        label="bot-spy")


def wait_for_lcm_message_on_channel(channel):
    """Wait for a single message to arrive on the specified LCM channel.
    """
    m = lcm.LCM()

    def receive(channel, data):
        _ = (channel, data)
        raise StopIteration()

    sub = m.subscribe(channel, receive)
    try:
        while True:
            rlist, _, _ = select.select([m], [], [])
            if m in rlist:
                m.handle()
    except StopIteration:
        pass
    finally:
        m.unsubscribe(sub)


def main():
    demo_name = "automotive_demo"
    demo_path = os.path.join(DRAKE_DRAKE_BIN_DIR, demo_name)

    parser = argparse.ArgumentParser(
        add_help=False, description=__doc__, epilog=_epilog % demo_name,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=float('Inf'),
                        help="demo run duration in seconds")
    parser.add_argument("--visualizer", action='store_true',
                        dest='launch_visualizer',
                        default=True, help="launch drake-visualizer (default)")
    parser.add_argument("--no-visualizer", action='store_false',
                        dest='launch_visualizer',
                        default=True, help="don't launch drake-visualizer")
    args, tail = parser.parse_known_args()

    if '--help' in tail:
        parser.print_help(file=sys.stderr)
        subprocess.call([demo_path, "--help"])
        sys.exit(1)

    try:
        # TODO(#3231) Use installed program once it works again.
        # the_launcher.launch([os.path.join(_DRAKE_INSTALL_BIN, "lcm-logger")])
        the_launcher.launch(
            [os.path.join(DRAKE_DIST_BUILD_DIR,
                          "externals", "lcm", "lcm-logger", "lcm-logger")])

        # TODO(#3231) Use this shell script once it works again.
        # the_launcher.launch(os.path.join(_DRAKE_INSTALL_BIN, "bot-spy")
        bot_spy_that_actually_works()

        if args.launch_visualizer:
            the_launcher.launch(
                [os.path.join(DRAKE_INSTALL_BIN_DIR, "drake-visualizer")])

            # Await a message on the DRAKE_VIEWER_STATUS channel indicating
            # that drake-visualizer is ready. This ensures that the demo app's
            # LOAD_ROBOT message will be seen and processed.
            wait_for_lcm_message_on_channel('DRAKE_VIEWER_STATUS')

        the_launcher.launch([demo_path] + tail)
        the_launcher.launch(
            [os.path.join(_THIS_DIR, "steering_command_driver.py")])

        the_launcher.wait(args.duration)

    finally:
        the_launcher.kill()

    sys.exit(the_launcher.returncode)

if __name__ == '__main__':
    main()
