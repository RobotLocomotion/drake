"""
Provides support for using ipywidgets with the systems framework, and a number
of useful widget systems.
"""

import asyncio
import sys
from IPython import get_ipython
from warnings import warn


# Note: The implementation below was inspired by
# https://github.com/Kirill888/jupyter-ui-poll , though I suspect it can be
# optimized.
#
# For reference,
# https://ipywidgets.readthedocs.io/en/latest/examples/Widget%20Asynchronous.html  # noqa
# describes the problem but it only offers the solution of using separate 
# threads for execution; a workflow that we do not wish to impose on users.

def process_ipywidget_events(num_events_to_process=1):
    """
    Allows the kernel to process GUI events.  This is required in order to
    process ipywidget updates inside a simulation loop.
    """

    shell = get_ipython()
    # Ok to do nothing if running from console
    if shell is None:
        return
    kernel = shell.kernel
    events = []
    kernel.shell_handlers['execute_request'] = lambda *e: events.append(e)
    current_parent = (kernel._parent_ident, kernel._parent_header)

    for _ in range(num_events_to_process):
        # ensure stdout still happens in the same cell
        kernel.set_parent(*current_parent)
        kernel.do_one_iteration()
        kernel.set_parent(*current_parent)

    kernel.shell_handlers['execute_request'] = kernel.execute_request

    def _replay_events(shell, events):
        kernel = shell.kernel
        sys.stdout.flush()
        sys.stderr.flush()
        for stream, ident, parent in events:
            kernel.set_parent(ident, parent)
            if kernel._aborting:
                kernel._send_abort_reply(stream, parent, ident)
            else:
                kernel.execute_request(stream, ident, parent)

    if len(events) > 0:
        loop = asyncio.get_event_loop()
        if loop.is_running():
            loop.call_soon(lambda: _replay_events(shell, events))
        else:
            warn('Automatic execution of scheduled cells only works with '
                'asyncio-based ipython')
