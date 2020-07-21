Cassie benchmark
----------------

This is a real-world example of a medium-sized robot with timing
tests for calculating its mass matrix, inverse dynamics, and
forward dynamics and their AutoDiff derivatives.

This gives us a straightforward way to measure improvements in
these basic multibody calculations in Drake.

As timings are performed, please note the date, the changes
made (with PR # if possible), the timings, and the specs
of the machine, os, compiler on which they were obtained. 
Warning: don't use virtual machines for timings; they seem 
unreliable.

#### 2020 July 29

Initial timings taken on Puget, Ubuntu 18.04.
Intel(R) Xeon(R) CPU E5-2650 v4 @ 2.20GHz (48 cpus)
L2 cache: 30720KB, 128GB RAM
g++ 7.5

```
Executing tests from //examples/multibody/cassie_benchmark:cassie_benchmark_test
-----------------------------------------------------------------------------
(multibody_plant) 100000x mass matrix took 2432 ms. 24 us per.
(multibody_plant) 1000x autodiff mass matrix took 2341 ms. 2341 us per.
(multibody_plant) 100000x inverse dynamics took 2353 ms. 23 us per.
(multibody_plant) 1000x autodiff inverse dynamics took 3260 ms. 3260 us per.
(multibody_plant) 100000x forward dynamics took 4639 ms. 46 us per.
(multibody_plant) 1000x autodiff forward dynamics took 4541 ms. 4541 us per.
```
These times were repeatable to +/-1 us per.
