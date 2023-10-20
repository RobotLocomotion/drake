"""Drake offers Python tutorials that can be previewed and executed as Jupyter
notebooks online with no need for local installation. To run Drake's tutorials
online, refer to the `Drake Tutorials <https://drake.mit.edu/>`_ website.

Alternatively, to run Drake's tutorials locally from an
`installed <https://drake.mit.edu/installation.html>`_ copy of Drake,
run ``python3 -m pydrake.tutorials`` to launch a Jupyter browser.

Be sure your ``PYTHONPATH`` has been set per the installation instructions,
e.g., via ``source env/bin/activate`` in the
`pip instructions <https://drake.mit.edu/pip.html>`_.

If you haven't done so already, you'll also need to install the Jupyter
notebook package on your system:

- For pip or macOS, use: ``pip install notebook``.
- For Ubuntu, use ``sudo apt-get install jupyter-notebook``.
"""


def __main():
    import sys
    import pydrake
    sys.argv = ["notebook", f"{pydrake.getDrakePath()}/tutorials/index.ipynb"]
    imported = False
    try:
        # Try the Jupyter >= 7 spelling first.
        from notebook import app
        imported = True
    except ImportError:
        pass
    if not imported:
        try:
            # Try the Jupyter < 7 spelling as a fallback.
            from notebook import notebookapp as app
            imported = True
        except ImportError:
            pass
    if not imported:
        print("ERROR: the Jupyter notebook runtime is not installed!")
        print()
        print(__doc__)
        sys.exit(1)
    app.launch_new_instance()


if __name__ == "__main__":
    __main()
