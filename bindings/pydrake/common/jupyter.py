"""
Provides support for using ipywidgets with the systems framework, and a number
of useful widget systems.

This is gui code; to test changes, please manually run
//bindings/pydrake/systems/jupyter_widgets_examples.ipynb.
"""

import asyncio
import sys
from warnings import warn

from IPython import get_ipython


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
    # Ok to do nothing if running from console.
    if shell is None or not hasattr(shell, 'kernel'):
        return
    kernel = shell.kernel
    events = []
    old_handler = kernel.shell_handlers['execute_request']
    kernel.shell_handlers['execute_request'] = lambda *e: events.append(e)
    current_parent = (kernel._parent_ident, kernel._parent_header)

    for _ in range(num_events_to_process):
        # Ensure stdout still happens in the same cell.
        kernel.set_parent(*current_parent)
        kernel.do_one_iteration()
        kernel.set_parent(*current_parent)

    kernel.shell_handlers['execute_request'] = old_handler

    def _replay_events(shell, events):
        kernel = shell.kernel
        sys.stdout.flush()
        sys.stderr.flush()
        for stream, ident, parent in events:
            kernel.set_parent(ident, parent)
            kernel.execute_request(stream, ident, parent)

    if len(events) > 0:
        loop = asyncio.get_event_loop()
        if loop.is_running():
            loop.call_soon(lambda: _replay_events(shell, events))
        else:
            warn("One of your components is attempting to use pydrake's "
                 "process_ipywidget_events function. However, this IPython "
                 "kernel is not asyncio-based. This means the following:\n"
                 "  (1) Once your block cell is done executing, future cells "
                 "will *not* execute, but it may appear like they are still "
                 "executing ([*]).\n"
                 "  (2) Your Jupyter UI may break. If you find your UI to be "
                 "unresponsive, you may need to restart the UI itself.\n"
                 "To avoid this behavior, avoid requesting execution of "
                 "future cells before or during execution of this cell.")
