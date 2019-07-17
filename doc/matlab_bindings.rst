.. _matlab-bindings:

***********************
Using Drake from MATLAB
***********************

MATLAB code/bindings available in Drake are unit tested on MATLAB R2017a on
Ubuntu 16.04 (Xenial Xerus).

We are currently experimenting with a few solutions for implementing the
MATLAB bindings for Drake's new C++ libraries.  You will find minimal
examples available in master, but most are not available yet.  Stay tuned
for more information.

In the meantime,
:ref:`Using Drake from Python <python-bindings>`
may also prove useful.

Building the MATLAB Bindings
----------------------------

.. code-block:: shell

    export PATH=/usr/local/MATLAB/R2017a/bin:$PATH  # Ubuntu
    git clone https://github.com/RobotLocomotion/drake.git
    mkdir drake-build
    cd drake-build
    cmake -DWITH_MATLAB=ON ../drake
    make

Troubleshooting
---------------

On Ubuntu, you may encounter an error similar to the following::

    /usr/local/MATLAB/R2017a/bin/glnxa64/../../sys/os/glnxa64/libstdc++.so.6: version `GLIBCXX_3.4.21' not found.

If so, modify your MATLAB installation as follows:

.. code-block:: shell

    cd /usr/local/MATLAB/R2017a/sys/os/glnxa64
    sudo rm libgfortran.so.3 libquadmath.so.0 libstdc++.so.6
    sudo ln -s /usr/lib/x86_64-linux-gnu/libgfortran.so.3 libgfortran.so.3
    sudo ln -s /usr/lib/x86_64-linux-gnu/libstdc++.so.6 libstdc++.so.6
    sudo ln -s /usr/lib/x86_64-linux-gnu/libquadmath.so.0 libquadmath.so.0
