#!/usr/bin/env python3

"""Tool to allow linux-perf sampling controlled by the target program.

"""

import argparse
import os
import shlex
import subprocess
import tempfile

# things to manage:
# sudo-ness
# perf record additional args
# target program command line
# working directory for recording
# env vars to communicate the fifos

def say(*args):
    """Print all the args, formatted for visibility."""
    print(f"\n=== {' '.join(args)} ===\n")


def sudo(*args, quiet=False, env=None):
    """Run sudo, passing all args to it."""
    new_args = ["sudo"] + (["-E"] if env is not None else []) + list(args)
    print('Running: ', shlex.join(new_args))
    if quiet:
        popen = subprocess.Popen(
            new_args, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            encoding='utf-8')
        if popen.wait() != 0:
            print(popen.stdout.read())
            raise RuntimeError("Failure during sudo()")
    else:
        subprocess.run(new_args, env=env, stderr=subprocess.STDOUT, check=True)


def do_recording(args):
    user = os.environ["USER"]

    with tempfile.TemporaryDirectory() as tmpdir:
        # Make fifos for perf<>target communication.
        ctl_fifo = os.path.join(tmpdir, 'ctl')
        os.mkfifo(ctl_fifo)
        ack_fifo = os.path.join(tmpdir, 'ack')
        os.mkfifo(ack_fifo)

        # Pass fifos to the target via env vars.
        perf_env = os.environ.copy()
        perf_env["DRAKE_PERF_CTL_FIFO"] = ctl_fifo
        perf_env["DRAKE_PERF_ACK_FIFO"] = ack_fifo

        # Pass fifos to perf via command line options.
        sudo("perf", "record", "--delay=-1",
             f"--control=fifo:{ctl_fifo},{ack_fifo}", "-g",
             *args.extra_perf_args,
             "--", "sudo", "-E", "-u", f"{user}", "--",
             *args.target_command_line, env=perf_env)

    sudo("chown", f"{user}.{user}", "perf.data")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--extra_perf_args', nargs='*', default=[],
        help='extra arguments passed to `perf record`')
    parser.add_argument(
        'target_command_line', nargs='*', default=[],
        help='target command line')
    args = parser.parse_args()

    do_recording(args)

if __name__ == '__main__':
    main()
