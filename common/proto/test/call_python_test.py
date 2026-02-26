"""Tests the `call_python_client` CLI and `call_python_server_test`
together."""

from contextlib import contextmanager
import os
import signal
import subprocess
import sys
import time
import unittest


@contextmanager
def scoped_file(filepath, is_fifo=False):
    # Ensures a file does not exist, creates it, and then destroys it upon
    # exiting the context.
    assert not os.path.exists(filepath)
    try:
        if is_fifo:
            os.mkfifo(filepath)
        else:
            with open(filepath, "w"):
                pass
        yield
    finally:
        os.unlink(filepath)


assert "TEST_TMPDIR" in os.environ, "Must run under `bazel test`"

# TODO(eric.cousineau): See if it's possible to make test usefully pass for
# "Template" backend.
os.environ["MPLBACKEND"] = "ps"

SIGPIPE_STATUS = 141

cur_dir = os.path.dirname(os.path.abspath(__file__))
# N.B. Need parent directories because this is under `test/*.py`, but the
# Bazel-generated script is one level above.
server_bin = os.path.join(cur_dir, "../call_python_server_test")
client_bin = os.path.join(cur_dir, "../call_python_client_cli")

file = os.path.join(os.environ["TEST_TMPDIR"], "python_rpc")
done_file = file + "_done"


def wait_for_done_count(num_expected, attempt_max=1000):
    done_count = -1
    attempt = 0
    values_read = set()
    while done_count < num_expected:
        with open(done_file) as f:
            try:
                done_count = int(f.read().strip())
            except ValueError:
                pass
        if done_count >= 0:
            values_read.add(done_count)
        time.sleep(0.005)
        attempt += 1
        if attempt == attempt_max:
            raise RuntimeError(
                "Did not get updated 'done count'. Read values: {}".format(
                    values_read
                )
            )


class TestCallPython(unittest.TestCase):
    def run_server_and_client(self, with_error):
        """Runs and tests server and client in parallel."""
        server_flags = ["--file=" + file, "--done_file=" + done_file]
        client_flags = ["--file=" + file]
        if with_error:
            server_flags += ["--with_error"]
            client_flags += ["--stop_on_error"]

        with scoped_file(file, is_fifo=True), scoped_file(done_file):
            with open(done_file, "w") as f:
                f.write("0\n")
            # Start client.
            client = subprocess.Popen([client_bin] + client_flags)
            # Start server.
            server = subprocess.Popen([server_bin] + server_flags)
            # Join with processes, check return codes.
            server_valid_statuses = [0]
            if with_error:
                # If the C++ binary has not finished by the time the Python
                # client exits due to failure, then the C++ binary will fail
                # with SIGPIPE.
                server_valid_statuses.append(SIGPIPE_STATUS)
            self.assertIn(server.wait(), server_valid_statuses)
            if not with_error:
                # Execute once more.
                server = subprocess.Popen([server_bin] + server_flags)
                self.assertIn(server.wait(), server_valid_statuses)
                # Wait until the client has indicated that the server process
                # has run twice. We want to run twice to ensure that looping on
                # the client end runs correctly.
                wait_for_done_count(2)
                client.send_signal(signal.SIGINT)
            client_status = client.wait()
            self.assertEqual(client_status, int(with_error))

    def test_help(self):
        text = subprocess.check_output(
            [client_bin, "--help"], stderr=subprocess.STDOUT
        ).decode("utf8")
        # Print output, since `assertIn` does not provide user-friendly
        # multiline error messages.
        print(text)
        self.assertTrue("Here's an example" in text)

    @unittest.skipIf(sys.platform == "darwin", "Flaky on macOS")
    def test_basic(self):
        for with_error in [False, True]:
            print("[ with_error: {} ]".format(with_error))
            self.run_server_and_client(with_error)
        # TODO(eric.cousineau): Cover other use cases if it's useful, or prune
        # them from the code.
