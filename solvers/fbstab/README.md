# FBstab
This directory contains an implementation of FBstab the proximally regularized Fischer-Burmeister method for quadratic programming. FBstab solves convex quadratic programs of the form,

```
min.  0.5 z'Hz + f'z
s.t.      Gz = h
          Az <= b
```

and its dual

```
min.  0.5 u'Hu + b'l + h'v
s.t.  Hu + f + G'l + A'v = 0
      v >= 0.
```

It assumes that H is positive semidefinite and can detect primal and/or dual infeasibility. FBstab requires no regularity assumptions on the G or A matrices. A mathematical description of FBstab can be found in the following [research article](https://arxiv.org/pdf/1901.04046.pdf).

## Organization
FBstab uses [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) as its linear algebra library and is designed to be extensible for different classes of QPs. Its architecture is similar to that of [OOQP](http://pages.cs.wisc.edu/~swright/ooqp/).

The algorithm itself is implemented in an abstract way using template metaprogramming. It requires 5 component classes which each must implement several methods.

1. A Variable class for storing primal-dual variables.
2. A Residual class for computing optimality residuals.
3. A Data class for storing and manipulating problem data.
4. A LinearSolver class for computing Newton steps.
5. A Feasibility class for detecting primal or dual infeasibility.

These classes should be specialized for different kinds of QPs, e.g., support vector machines, optimal control problems, general dense QPs, in order to exploit structure. FBstab currently supports:

- Linear-quadratic optimal control problems with polyhedral constraints, i.e., model predictive control problems.
- Dense inequality constrained problems.





