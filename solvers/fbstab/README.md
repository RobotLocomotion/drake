# FBstab
This directory contains and implementation of FBstab
the proximally regularized Fischer-Burmeister method for quadratic programming. FBstab solves convex quadratic programs of the form,

```
min.  0.5 z'Hz + f'z  
s.t.      Gz = h  
          Az <= b
```

It assumes only that H is positive semidefinite and can detect primal and dual infeasibility. A mathematical description of FBstab can be found in the following [research article](https://arxiv.org/pdf/1901.04046.pdf).

## Orginization
FBstab uses Eigen as its linear algebra library and is designed to be extensible for different classes of QPs. Its architecture is similar to that of [OOQP](http://pages.cs.wisc.edu/~swright/ooqp/).

The algorithm itself is implemented in a abstract way using template metaprogramming. It requres 5 classes which each must implement several methods.

1. A Variable class for storing primal-dual variables
2. A Residual class for computing optimality residuals
3. A Data class for storing and manipulating problem data
4. A linear solver class for computing Newton steps
5. A feasibility class for detecting primal or dual infeasibility 

These classes should be specialized to different kinds of QPs, e.g., support vector machines, optimal control problems, general dense QPs, in order to exploit structure. 

