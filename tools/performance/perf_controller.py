"""Instrumentation helpers for in-process control of profile sampling by Linux
`perf`. See README.md. for an overview.
"""

from contextlib import contextmanager
import os

_enable_cmd = "enable".encode('utf-8')
_disable_cmd = "disable".encode('utf-8')
_ack_cmd = "ack\n\0".encode('utf-8')


def _fd_from_env(env_key: str) -> int:
    name = os.environ.get(env_key)
    if not name:
        return -1
    return os.open(name, os.O_RDWR)


class _PerfController:
    """In-process controller class.

    This class is designed to work together with the top-level launch script
    perf_controlled_record.py. It looks for the names of two FIFO files via the
    environment variables DRAKE_PERF_CTL_FIFO and DRAKE_PERF_ACK_FIFO. If those
    are missing, or the files can't be opened, then control will be
    unavailable.

    No attempt is made to make this class thread-safe, so be careful to avoid
    issuing commands from multiple threads.
    """

    def __init__(self):
        self._ctl_fd = _fd_from_env("DRAKE_PERF_CTL_FIFO")
        self._ack_fd = _fd_from_env("DRAKE_PERF_ACK_FIFO")
        self._is_control_available = (self._ctl_fd >= 0 and self._ack_fd >= 0)

    def pause(self):
        """Turn sampling off."""
        if not self.is_control_available():
            return
        self._send_command(_disable_cmd)

    def resume(self):
        """ Turn sampling on."""
        if not self.is_control_available():
            return
        self._send_command(_enable_cmd)

    def is_control_available(self):
        """Returns True iff sampling control is available."""
        return self._is_control_available

    def _send_command(self, command):
        if not self.is_control_available():
            return
        wrote_bytes = os.write(self._ctl_fd, command)
        if wrote_bytes != len(command):
            raise RuntimeError("`perf` command not completely written.")
        got = os.read(self._ack_fd, len(_ack_cmd))
        if got != _ack_cmd:
            raise RuntimeError(
                f"`perf` command acknowledgment not received:"
                f" {got} != {_ack_cmd}")


_the_perf_controller = _PerfController()


def ThePerfController():
    """Returns the singleton PerfController."""
    return _the_perf_controller


@contextmanager
def scoped_perf_sampling():
    """Context manager to turn sampling on for the duration
    of a section of code.
    """
    ThePerfController().resume()
    try:
        yield
    finally:
        ThePerfController().pause()
