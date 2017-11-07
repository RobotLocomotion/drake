.. _matlab-bindings:

***********************
Using Drake from MATLAB
***********************

MATLAB code/bindings available in Drake are unit tested on MATLAB R2017a.
Other MATLAB versions R2016a or above may work but are not officially
supported.

We are currently experimenting with a few solutions for implementing the
MATLAB bindings for Drake's new C++ libraries.  You will find minimal
examples available in master, but most are not available yet.  Stay tuned
for more information.

In the meantime,
:ref:`Using Drake from Python <python-bindings>`
may also prove useful.

Building the MATLAB Bindings
----------------------------

    export PATH=/usr/local/MATLAB/R2017a/bin:$PATH  # Ubuntu
    git clone https://github.com/RobotLocomotion/drake.git
    mkdir drake-build
    cd drake-build
    cmake -DWITH_MATLAB=ON ../drake
    make

Original MATLAB
===============

Prior to 2016, Drake was built around a substantial base of MATLAB software.
Most of that was removed from the head of git master during 2017.

To view or use the original MATLAB implementation, you may use this tag:

https://github.com/RobotLocomotion/drake/tree/last_sha_with_original_matlab
