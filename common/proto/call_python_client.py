"""
Permits calling arbitrary functions and passing some forms of data from C++
to Python (only one direction) as a server-client pair.

The server in this case is the C++ program, and the client is this binary.
For an example of C++ usage, see `call_python_server_test.cc`.

Here's an example of running with the C++ test program:

    cd drake
    bazel build //common/proto:call_python_client_cli //common/proto:call_python_server_test  # noqa
    # Create default pipe file.
    rm -f /tmp/python_rpc && mkfifo /tmp/python_rpc

    # In Terminal 1, run client.
    ./bazel-bin/common/proto/call_python_client_cli

    # In Terminal 2, run server (or your C++ program).
    ./bazel-bin/common/proto/call_python_server_test

To use in Jupyter (if you have it installed) without a FIFO file (such that
it's non-blocking):

    cd drake
    bazel build //common/proto:call_python_client_cli //common/proto:call_python_server_test  # noqa
    rm -f /tmp/python_rpc  # Do not make it FIFO

    # In Terminal 1, run server, create output.
    ./bazel-bin/common/proto/call_python_server_test

    # In Terminal 2, run client in notebook.
    ./bazel-bin/common/proto/call_python_client_cli \
        -c jupyter notebook ${PWD}/common/proto/call_python_client_notebook.ipynb  # noqa
    # Execute: Cell > Run All

Note:
    Occasionally, the plotting will not come through on the notebook. I (Eric)
    am unsure why.
"""

import argparse
import os
from queue import Queue
import signal
import stat
import sys
from threading import Thread
import time
import traceback

import numpy as np

from drake import lcmt_call_python, lcmt_call_python_data


def _ensure_sigint_handler():
    # @ref https://stackoverflow.com/a/47801921/2654527
    if signal.getsignal(signal.SIGINT) == signal.SIG_IGN:
        signal.signal(signal.SIGINT, signal.default_int_handler)


def _get_required_helpers(scope_locals):
    # Provides helpers to keep C++ interface as simple as possible.
    # @returns Dictionary containing the helpers needed.
    def getitem(obj, index):
        """Global function for `obj[index]`. """
        return obj[index]

    def setitem(obj, index, value):
        """Global function for `obj[index] = value`. """
        obj[index] = value
        return obj[index]

    def call(obj, *args, **kwargs):
        return obj(*args, **kwargs)

    def pass_through(value):
        """Pass-through for direct variable access. """
        return value

    def make_tuple(*args):
        """Create a tuple from an argument list. """
        return tuple(args)

    def make_list(*args):
        """Create a list from an argument list. """
        return list(args)

    def make_kwargs(*args):
        """Create a keyword argument object from an argument list. """
        assert len(args) % 2 == 0
        keys = args[0::2]
        values = args[1::2]
        kwargs = dict(zip(keys, values))
        return _KwArgs(**kwargs)

    def _make_slice(expr):
        """Parse a slice object from a string. """
        def to_piece(s):
            return s and int(s) or None
        pieces = list(map(to_piece, expr.split(':')))
        if len(pieces) == 1:
            return slice(pieces[0], pieces[0] + 1)
        else:
            return slice(*pieces)

    def make_slice_arg(*args):
        """Create a scalar or tuple for accessing objects via slices. """
        out = [None] * len(args)
        for i, arg in enumerate(args):
            if isinstance(arg, str):
                out[i] = _make_slice(arg)
            else:
                out[i] = arg
        # Special case: If single index, collapse.
        if len(out) == 1:
            return out[0]
        else:
            return tuple(out)

    def setvar(var, value):
        """Sets a variable in the client's locals. """
        scope_locals[var] = value

    def setvars(*args):
        """Sets multiple variables in the client's locals. """
        scope_locals.update(make_kwargs(*args))

    execution_check = _ExecutionCheck()

    out = locals().copy()
    # Scrub extra stuff.
    del out["scope_locals"]
    return out


class _KwArgs(dict):
    # Indicates values meant solely for `**kwargs`.
    pass


class _ExecutionCheck:
    # Allows checking that we received and executed a complete set of
    # instructions.
    def __init__(self):
        self.count = 0

    def start(self):
        self.count += 1

    def finish(self):
        assert self.count > 0
        self.count -= 1


def _merge_dicts(*args):
    # Merges a list of dict's.
    out = {}
    for arg in args:
        out.update(arg)
    return out


def _fix_pyplot(plt):
    # This patches matplotlib/matplotlib#9412 by injecting `time` into the
    # module (#7597).
    cur = plt.__dict__
    if 'time' not in cur:
        cur['time'] = time


def default_globals():
    """Creates default globals for code that the client side can execute.

    This is geared for convenient (not necessarily efficient) plotting
    with `matplotlib`.
    """
    # @note This imports modules at a function-scope rather than at a
    # module-scope, which does not satisfy PEP8. This is intentional, as it
    # allows for a cleaner scope separation between the client core code (e.g.
    # `CallPythonClient`) and the client user code (e.g. `plot(x, y)`).
    # TODO(eric.cousineau): Consider relegating this to a different module,
    # possibly when this falls under `pydrake`.
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib
    import matplotlib.pyplot as plt
    import pylab  # See `%pylab?` in IPython.

    # TODO(eric.cousineau): Where better to put this?
    matplotlib.interactive(True)
    _fix_pyplot(plt)

    def disp(value):
        """Alias for print."""
        print(value)

    def wait():
        """Waits to allow user interaction with plots."""
        plt.show(block=True)

    def pause(interval):
        """Pause for `interval` seconds, letting the GUI flush its event queue.

        @note This is a *necessary* function to be defined if these globals are
        not used!
        """
        plt.pause(interval)

    def box(bmin, bmax, rstride=1, cstride=1, **kwargs):
        """Plots a box bmin[i] <= x[i] <= bmax[i] for i < 3."""
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        u = np.linspace(1, 9, 5) * np.pi / 4
        U, V = np.meshgrid(u, u)
        cx, cy, cz = (bmax + bmin) / 2
        dx, dy, dz = bmax - bmin
        X = cx + dx * np.cos(U) * np.sin(V)
        Y = cy + dy * np.sin(U) * np.sin(V)
        Z = cz + dz * np.cos(V) / np.sqrt(2)
        ax.plot_surface(X, Y, Z, rstride=rstride, cstride=cstride, **kwargs)

    def plot3(x, y, z, **kwargs):
        """Plots a 3d line plot."""
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        ax.plot(x, y, z, **kwargs)

    def sphere(n, rstride=1, cstride=1, **kwargs):
        """Plots a sphere."""
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        u = np.linspace(0, np.pi, n)
        v = np.linspace(0, 2 * np.pi, n)
        X = np.outer(np.sin(u), np.sin(v))
        Y = np.outer(np.sin(u), np.cos(v))
        Z = np.outer(np.cos(u), np.ones_like(v))
        ax.plot_surface(X, Y, Z, rstride=rstride, cstride=cstride, **kwargs)

    def surf(x, y, Z, rstride=1, cstride=1, **kwargs):
        """Plots a 3d surface."""
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        X, Y = np.meshgrid(x, y)
        ax.plot_surface(X, Y, Z, rstride=rstride, cstride=cstride, **kwargs)

    def show():
        """Shows `matplotlib` images without blocking.

        Generally not needed if `matplotlib.is_interactive()` is true.
        """
        plt.show(block=False)

    def magic(N):
        """Provides simple odd-only case for magic squares.

        @ref https://scipython.com/book/chapter-6-numpy/examples/creating-a-magic-square  # noqa
        """
        assert N % 2 == 1
        magic_square = np.zeros((N, N), dtype=int)
        n = 1
        i, j = 0, N//2
        while n <= N**2:
            magic_square[i, j] = n
            n += 1
            newi, newj = (i - 1) % N, (j + 1) % N
            if magic_square[newi, newj]:
                i += 1
            else:
                i, j = newi, newj
        return magic_square

    # Use <module>.__dict__ to simulate `from <module> import *`, since that is
    # normally invalid in a function with nested functions.
    return _merge_dicts(
        globals(),
        plt.__dict__,
        pylab.__dict__,
        locals())


class CallPythonClient:
    """Provides a client to receive Python commands.

    Enables printing or plotting from a C++ application for debugging
    purposes.
    """
    def __init__(self, filename=None, stop_on_error=True,
                 scope_globals=None, scope_locals=None,
                 threaded=False, wait=False):
        if filename is None:
            # TODO(jamiesnape): Implement and use a
            # drake.common.GetRpcPipeTempDirectory function.
            temp_directory = os.environ.get("TEST_TMPDIR", "/tmp")
            self.filename = os.path.join(temp_directory, "python_rpc")
        else:
            self.filename = filename
        # Scope. Give it access to everything here.
        # However, keep it's written values scoped.
        if scope_locals is None:
            self.scope_locals = {}
        else:
            self.scope_locals = scope_locals
        # Define globals as (a) required helpers for C++ interface, and
        # (b) convenience plotting functionality.
        # N.B. The provided locals OR globals can shadow the helpers. BE
        # CAREFUL!
        required_helpers = _get_required_helpers(self.scope_locals)
        if scope_globals is None:
            scope_globals = default_globals()
        self.scope_globals = _merge_dicts(required_helpers, scope_globals)

        self._stop_on_error = stop_on_error
        self._threaded = threaded

        self._loop = False
        self._wait = False
        if wait:
            if _is_fifo(self.filename):
                self._loop = True
                print("Looping for FIFO file (wait=True).")
            else:
                self._wait = True
                print("Waiting after processing non-FIFO file (wait=True).")

        # Variables indexed by GUID.
        self._client_vars = {}

        self._had_error = False
        self._done = False
        self._file = None

    def _to_array(self, arg, dtype):
        # Converts a lcmt_call_python argument to the appropriate NumPy array
        # (or scalar).
        np_raw = np.frombuffer(arg.data, dtype=dtype)
        if arg.shape_type == lcmt_call_python_data.SCALAR:
            assert arg.cols == 1 and arg.rows == 1
            return np_raw[0]
        elif arg.shape_type == lcmt_call_python_data.VECTOR:
            assert arg.cols == 1
            return np_raw.reshape(arg.rows)
        elif arg.shape_type is None or \
                arg.shape_type == lcmt_call_python_data.MATRIX:
            # TODO(eric.cousineau): Figure out how to ensure `np.frombuffer`
            # creates a column-major array?
            return np_raw.reshape(arg.cols, arg.rows).T

    def _execute_message(self, msg):
        # Executes a message, handling / recording that an error occurred.
        if self._stop_on_error:
            # Do not wrap in a `try` / `catch` to simplify debugging.
            self._execute_message_impl(msg)
        else:
            try:
                self._execute_message_impl(msg)
            except Exception as e:
                traceback.print_exc(file=sys.stderr)
                sys.stderr.write("  Continuing (no --stop_on_error)\n")
                self._had_error = True

    def _execute_message_impl(self, msg):
        # Executes relevant portions of a message.
        # Create input arguments.
        inputs = []
        kwargs = None
        for i, arg in enumerate(msg.rhs):
            value = None
            if (arg.data_type
                    == lcmt_call_python_data.REMOTE_VARIABLE_REFERENCE):
                id = np.frombuffer(arg.data, dtype=np.uint64).reshape(1)[0]
                if id not in self._client_vars:
                    raise RuntimeError("Unknown local variable. "
                                       "Dropping message.")
                value = self._client_vars[id]
            elif arg.data_type == lcmt_call_python_data.DOUBLE:
                value = self._to_array(arg, np.double)
            elif arg.data_type == lcmt_call_python_data.CHAR:
                assert arg.rows == 1
                value = arg.data.decode('utf8')
            elif arg.data_type == lcmt_call_python_data.LOGICAL:
                value = self._to_array(arg, np.bool)
            elif arg.data_type == lcmt_call_python_data.INT:
                value = self._to_array(arg, np.int32)
            else:
                assert False
            if isinstance(value, _KwArgs):
                assert kwargs is None
                kwargs = value
            else:
                inputs.append(value)

        # Call the function
        # N.B. No security measures to sanitize function name.
        function_name = msg.function_name
        assert isinstance(function_name, str), type(function_name)

        self.scope_locals.update(_tmp_args=inputs, _tmp_kwargs=kwargs or {})
        # N.B. No try-catch block here. Can change this if needed.
        if function_name == "exec":
            assert len(inputs) == 1
            assert kwargs is None or len(kwargs) == 0
            exec(inputs[0], self.scope_globals, self.scope_locals)
            out = None
        else:
            out = eval(function_name + "(*_tmp_args, **_tmp_kwargs)",
                       self.scope_globals, self.scope_locals)
        self.scope_locals.update(_tmp_out=out)
        # Update outputs.
        self._client_vars[msg.lhs] = out

    def run(self):
        """Runs the client code.

        @return True if no error encountered.
        """
        if self._threaded:
            self._handle_messages_threaded()
        else:
            self.handle_messages(record=False)
        # Check any execution in progress.
        execution_check = self.scope_globals['execution_check']
        if not self._had_error and execution_check.count != 0:
            self._had_error = True
            sys.stderr.write(
                "ERROR: Invalid termination. "
                "'execution_check.finish' called insufficient number of "
                "times: {}\n".format(execution_check.count))
        if self._wait and not self._had_error:
            wait_func = self.scope_globals["wait"]
            wait_func()
        return not self._had_error

    def _handle_messages_threaded(self):
        # Handles messages in a threaded fashion.
        queue = Queue()

        def producer_loop():
            # Read messages from file, and queue them for execution.
            for msg in self._read_next_message():
                queue.put(msg)
                # Check if an error occurred.
                if self._done:
                    break
            # Wait until the queue empties out to signal completion from the
            # producer's side.
            if not self._done:
                queue.join()
                self._done = True

        producer = Thread(name="Producer", target=producer_loop)
        # @note Previously, when trying to do `queue.clear()` in the consumer,
        # and `queue.join()` in the producer, there would be intermittent
        # deadlocks. By demoting the producer to a daemon, I (eric.c) have not
        # yet encountered a deadlock.
        producer.daemon = True
        producer.start()

        # Consume.
        # TODO(eric.cousineau): Trying to quit via Ctrl+C is awkward (but kinda
        # works). Is there a way to have `plt.pause` handle Ctrl+C differently?
        try:
            pause = self.scope_globals['pause']
            while not self._done:
                # Process messages.
                while not queue.empty():
                    msg = queue.get()
                    queue.task_done()
                    self._execute_message(msg)
                # Spin busy for a bit, let matplotlib (or whatever) flush its
                # event queue.
                pause(0.01)
        except KeyboardInterrupt:
            # User pressed Ctrl+C.
            self._done = True
            print("Quitting")
        except Exception as e:
            # We encountered an error, and must stop.
            self._done = True
            self._had_error = True
            traceback.print_exc(file=sys.stderr)
            sys.stderr.write("  Stopping (--stop_on_error)\n")
        # No need to worry about waiting for the producer, as it is a daemon
        # thread.

    def handle_messages(self, max_count=None, record=True, execute=True):
        """Handle all messages sent (e.g., through IPython).
        @param max_count Maximum number of messages to handle.
        @param record Record all messages and return them.
        @param execute Execute the given message upon receiving it.
        @return (count, msgs) where `count` is how many messages were processed
        (e.g. 0 if no more messages left).
        and `msgs` are either the messages themselves for playback.
        and (b) the messages themselves for playback (if record==True),
        otherwise an empty list.
        """
        assert record or execute, "Not doing anything useful?"
        count = 0
        msgs = []
        for msg in self._read_next_message():
            if execute:
                self._execute_message(msg)
            count += 1
            if record:
                msgs.append(msg)
            if max_count is not None and count >= max_count:
                break
        return (count, msgs)

    def execute_messages(self, msgs):
        """Executes a set of recorded messages."""
        for msg in msgs:
            self._execute_message(msg)

    def _read_next_message(self):
        """Returns incoming messages using a generator."""
        while not self._done:
            fifo = self._get_file()
            # Close the file if we reach the end, NOT when exiting the scope
            # (which is why `with` is not used here).
            # This way the user can read a few messages at a time, with the
            # same file handle.
            # @note We must close / reopen the file when looping because the
            # C++ program will effectively send a EOF signal when it closes
            # the pipe.
            while not self._done:
                message = self._read_fifo_message(fifo)
                if message is not None:
                    yield message
            self._close_file()
            if not self._loop:
                break

    def _read_fifo_message(self, fifo):
        """Reads at most one message from the given fifo."""
        # Read the datagram size.  (The C++ code encodes the datagram_size
        # integer as an ASCII string.)
        datagram_size = None
        buffer = bytearray()
        while not self._done:
            byte = fifo.read(1)
            if not byte:  # EOF
                return None
            if byte == b'\0':  # EOM
                datagram_size = int(buffer.decode())
                break
            else:
                buffer.extend(byte)

        # Read the payload.
        buffer[:] = ()
        while not self._done:
            byte = fifo.read(1)
            if not byte:  # EOF
                return None
            buffer.extend(byte)
            if len(buffer) == datagram_size:
                byte = fifo.read(1)
                assert byte == b'\0'  # EOM
                return lcmt_call_python.decode(bytes(buffer))

    def _get_file(self):
        # Gets file handle, opening if needed.
        if self._file is None:
            self._file = open(self.filename, 'rb')
        return self._file

    def _close_file(self):
        # Closes file if open.
        if self._file is not None:
            self._file.close()
            self._file = None


def _is_fifo(filepath):
    # Determine if a file is a FIFO named pipe or not.
    # @ref https://stackoverflow.com/a/8558940/7829525
    return stat.S_ISFIFO(os.stat(filepath).st_mode)


def main(argv):
    _ensure_sigint_handler()
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--no_wait", action='store_true',
        help="Close client after messages are processed. "
             "For FIFO, this means the client will close after the C++ "
             "binary is executed once.")
    parser.add_argument(
        "--no_threading", action='store_true',
        help="Disable threaded dispatch.")
    parser.add_argument(
        "--stop_on_error", action='store_true',
        help="Stop client if there is an error when executing a call.")
    parser.add_argument("-f", "--file", type=str, default=None)
    parser.add_argument(
        "-c", "--command", type=str, nargs='+', default=None,
        help="Execute command (e.g. `jupyter notebook`) instead of running "
             "client.")
    args = parser.parse_args(argv)

    if args.command is not None:
        # Execute command s.t. it has access to the relevant PYTHNOPATH.
        os.execvp(args.command[0], args.command)
        # Control should not return to this program unless there was an error.
        return False
    else:
        client = CallPythonClient(
            args.file, stop_on_error=args.stop_on_error,
            threaded=not args.no_threading, wait=not args.no_wait)
        good = client.run()
        return good


if __name__ == "__main__":
    good = main(sys.argv[1:])
    if not good:
        exit(1)
