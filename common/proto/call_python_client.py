#!/usr/bin/env python

from __future__ import print_function
import copy
import numpy as np
import sys
from threading import Thread, Lock

# Hacky, but this is the simplest route right now.
# @ref https://www.datadoghq.com/blog/engineering/protobuf-parsing-in-python/
from google.protobuf.internal.decoder import _DecodeVarint32

from drake.common.proto.matlab_rpc_pb2 import MatlabArray, MatlabRPC


def _get_required_helpers(scope_locals):
    # Provides helpers to keep C++ interface as simple as possible.
    # @returns Dictionary containing the helpers needed.
    def getitem(obj, index):
        """ Global function for `obj[index]`. """
        return obj[index]

    def setitem(obj, index, value):
        """ Global function for `obj[index] = value`. """
        obj[index] = value
        return obj[index]

    def call(obj, *args, **kwargs):
        return obj(*args, **kwargs)

    def pass_through(value):
        """ Pass-through for direct variable access. """
        return value

    def make_tuple(*args):
        """ Create a tuple from an argument list. """
        return tuple(args)

    def make_list(*args):
        """ Create a list from an argument list. """
        return list(args)

    def make_kwargs(*args):
        """ Create a keyword argument object from an argument list. """
        assert len(args) % 2 == 0
        keys = args[0::2]
        values = args[1::2]
        kwargs = dict(zip(keys, values))
        return _KwArgs(**kwargs)

    def _make_slice(expr):
        """ Parse a slice object from a string. """
        def to_piece(s):
            return s and int(s) or None
        pieces = map(to_piece, expr.split(':'))
        if len(pieces) == 1:
            return slice(pieces[0], pieces[0] + 1)
        else:
            return slice(*pieces)

    def make_slice_arg(*args):
        """ Create a scalar or tuple for acessing objects via slices. """
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
        """ Sets a variable in the client's locals. """
        scope_locals[var] = value

    def setvars(*args):
        """ Sets multiple variables in the client's locals. """
        scope_locals.update(make_kwargs(*args))

    out = locals().copy()
    # Scrub extra stuff.
    del out["scope_locals"]
    return out


class _KwArgs(dict):
    # Indicates values meant solely for `**kwargs`.
    pass


def _merge_dicts(*args):
    # Merges a list of dict's.
    out = {}
    for arg in args:
        out.update(arg)
    return out


def default_globals():
    """ Creates default globals for code that the client side can execute.
    This is geared for convenient (not necessarily efficient) plotting
    with `matplotlib`. """
    import numpy as np
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib
    import matplotlib.pyplot as plt
    import pylab  # See `%pylab?` in IPython.

    # Where better to put this?
    matplotlib.interactive(True)

    def disp(value):
        """ Alias for print. """
        print(value)

    def wait():
        """ Wait to allow user interaction with plots. """
        plt.show(block=True)

    def surf(x, y, Z, rstride=1, cstride=1, **kwargs):
        """ Plot a 3d surface. """
        fig = plt.gcf()
        ax = fig.gca(projection='3d')
        X, Y = np.meshgrid(x, y)
        ax.plot_surface(X, Y, Z, rstride=rstride, cstride=cstride, **kwargs)

    def show():
        """ Show `matplotlib` images without blocking.
        Generally not needed if `matplotlib.is_interactive()` is true. """
        plt.show(block=False)

    def magic(N):
        """ Simple odd-only case for magic squares.
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


_FILENAME_DEFAULT = "/tmp/python_rpc"


class CallPythonClient(object):
    """ Provides a client to receive Python commands (e.g. printing or
    plotting) from a C++ server for debugging purposes. """
    def __init__(self, filename=None, stop_on_error=True,
                 scope_globals=None, scope_locals=None):
        if filename is None:
            self.filename = _FILENAME_DEFAULT
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

        # Variables indexed by GUID.
        self._client_vars = {}

        self._had_error = False

    def _to_array(self, arg, dtype):
        # Convert a protobuf argument to the appropriate NumPy array (or
        # scalar).
        np_raw = np.frombuffer(arg.data, dtype=dtype)
        if arg.shape_type == MatlabArray.SCALAR:
            assert arg.cols == 1 and arg.rows == 1
            return np_raw[0]
        elif arg.shape_type == MatlabArray.VECTOR:
            assert arg.cols == 1
            return np_raw.reshape(arg.rows)
        elif arg.shape_type is None or arg.shape_type == MatlabArray.MATRIX:
            # TODO(eric.cousineau): Figure out how to ensure `np.frombuffer`
            # creates a column-major array?
            return np_raw.reshape(arg.cols, arg.rows).T

    def _execute_message(self, msg):
        if self._stop_on_error:
            # Do not wrap in a `try` / `catch` to simplify debugging.
            self._execute_message_impl(msg)
        else:
            try:
                self._execute_message_impl(msg)
            except Exception as e:
                sys.stderr.write("ERROR: {}\n".format(e))
                self._had_error = True

    def _execute_message_impl(self, msg):
        # Executes relevant portions of a message.
        # Create input arguments.
        args = msg.rhs
        nargs = len(args)
        inputs = []
        kwargs = None
        for i, arg in enumerate(args):
            arg_raw = arg.data
            value = None
            if arg.type == MatlabArray.REMOTE_VARIABLE_REFERENCE:
                id = np.frombuffer(arg_raw, dtype=np.uint64).reshape(1)[0]
                if id not in self._client_vars:
                    raise RuntimeError("Unknown local variable. " +
                                       "Dropping message.")
                value = self._client_vars[id]
            elif arg.type == MatlabArray.DOUBLE:
                value = self._to_array(arg, np.double)
            elif arg.type == MatlabArray.CHAR:
                assert arg.rows == 1
                value = str(arg_raw)
            elif arg.type == MatlabArray.LOGICAL:
                value = self._to_array(arg, np.bool)
            elif arg.type == MatlabArray.INT:
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
        out_id = None
        if len(msg.lhs) > 0:
            assert len(msg.lhs) == 1
            out_id = msg.lhs[0]

        self.scope_locals.update(_tmp_args=inputs, _tmp_kwargs=kwargs or {})
        # N.B. No try-catch block here. Can change this if needed.
        out = eval(function_name + "(*_tmp_args, **_tmp_kwargs)",
                   self.scope_globals, self.scope_locals)
        self.scope_locals.update(_tmp_out=out)
        # Update outputs.
        self._client_vars[out_id] = out

    def run(self):
        """ Runs the client code.
        @return True if no error encountered. """
        self.handle_messages(record=False)
        return not self._had_error

    def handle_messages(self, max_count=None, record=True, execute=True):
        """ Handle all messages sent (e.g., through IPython).
        @param max_count Maximum number of messages to handle.
        @param record Record all messages and return them.
        @param execute Execute the given message upon receiving it.
        @return (count, msgs) where `count` is how many messages were processed
        (e.g. 0 if no more messages left).
        and `msgs` are either the messages themselves for playback.
        and (b) the messages themselves for playback (if record==True),
        otherwise an empty list. """
        assert record or execute, "Not doing anything useful?"
        count = 0
        msgs = []
        for msg in self._read_next_message():
            if execute:
                self._execute_message(msg)
            count += 1
            if record:
                msgs.append(copy.deepcopy(msg))
            if max_count is not None and count >= max_count:
                break
        return (count, msgs)

    def execute_messages(self, msgs):
        """ Execute a set of recorded messages. """
        for msg in msgs:
            self._execute_message(msg)

    def _read_next_message(self):
        # Return a new incoming message using a generator.
        # Not guaranteed to be a unique instance. Should copy if needed.
        msg = MatlabRPC()
        with open(self.filename, 'rb') as f:
            while _read_next(f, msg):
                yield msg


# Number of bytes we need to consume so that we may still use
# `_DecodeVarint32`.
_PEEK_SIZE = 4


def _read_next(f, msg):
    # Read next message from the given file, following suite with C++.
    peek = f.read(_PEEK_SIZE)
    if len(peek) == 0:
        # We have reached the end.
        return 0
    msg_size, peek_end = _DecodeVarint32(peek, 0)
    peek_left = _PEEK_SIZE - peek_end
    # Read remaining and concatenate.
    remaining = f.read(msg_size - peek_left)
    msg_raw = peek[peek_end:] + remaining
    assert len(msg_raw) == msg_size
    # Now read the message.
    msg.ParseFromString(msg_raw)
    return msg_size


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--no_loop", action='store_true',
        help="Do not loop the end for user interaction. With a FIFO pipe, " +
             "this will end session after C++ program closes.")
    parser.add_argument("--stop_on_error", action='store_true',
                        help="Stop client if there is an error when " +
                             "executing a call.")
    parser.add_argument("-f", "--file", type=str, default=None)
    args = parser.parse_args(sys.argv[1:])

    client = CallPythonClient(args.file, stop_on_error=args.stop_on_error)
    good = client.run()
    if not args.no_loop:
        wait = client.scope_globals["wait"]
        print("Waiting... Ctrl+C may not work.")
        print("  Use Ctrl+\\ to forcefully abort.")
        wait()
    if not good:
        exit(1)
