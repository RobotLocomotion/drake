.. _matlab-bindings:

***********************
Using Drake from Matlab
***********************

We are currently implementing the MATLAB bindings for Drake's new C++ libraries.
Stay tuned for more information.

In the meantime,
:ref:`Using Drake from Python <python-bindings>`
may also prove useful.

See also
========

.. toctree::
    :maxdepth: 1

    matlab_faq

Using MATLAB with Drake on Ubuntu
=================================

MATLAB packages a very old version of ``libstdc++`` in the ``sys/os/glnxa64``
directory.  If you call any of the mex files built with drake, this old
version of ``libstdc++`` will conflict and cause MATLAB to crash.  Start
with::

  cd /usr/local/MATLAB/R2017a/sys/os/glnxa64
  ls -l

where you may need to update the path above to your MATLAB install directory.
You will see that ``libstdc++.so.6`` is already just a symlink; changing it
is safe.  Now find your system ``libstdc++`` replacement, probably in::

  ls -l /usr/lib/x86_64-linux-gnu/libstdc++.so.6

Now update MATLAB's symlink::

  sudo rm libstdc++.so.6
  sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 .

Restart MATLAB if you had any sessions open for the changes to take effect.
  

Original MATLAB
===============

Prior to 2016, Drake was built around a substantial base of MATLAB software.
Most of that was removed from the head of git master during 2017.

To view or use the original MATLAB implementation, you may use this tag:

https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab
