function [str,category] = snoptInfo(info)

switch (floor(info/10))
  case 0
    category='Finished successfully';
  case 1
    category='The problem appears to be infeasible';
  case 2
    category='The problem appears to be unbounded';
  case 3
    category='Resource limit error';
  case 4
    category='Terminated after numerical difficulties';
  case 5
    category='Error in the user-supplied functions';
  case 6
    category='Undefined user-supplied functions';
  case 7
    category='User requested termination';
  case 8
    category='Insufficient storage allocated';
  case 9
    category='Input arguments out of range';
  case 14
    category='System error';
end

switch (info)
  case 1 
    str='optimality conditions satisfied';
  case 2 
    str='feasible point found';
  case 3 
    str='requested accuracy could not be achieved';
  case 11
    str='infeasible linear constraints';
  case 12
    str='infeasible linear equalities';
  case 13
    str='nonlinear infeasibilities minimized';
  case 14
    str='infeasibilities minimized';
  case 15
    str='infeasible linear constraints in QP';
  case 21
    str='unbounded objective';
  case 22
    str='constraint violation limit reached';
  case 31
    str='iteration limit reached';
  case 32 
    str='major iteration limit reached';
  case 33
    str='the superbasics limit is too small';
  case 41
    str='current point cannot be improved';
  case 42
    str='singular basis';
  case 43
    str='cannot satisfy the general constraints';
  case 44
    str='ill-conditioned null-space basis';
  case 51
    str='incorrect objective derivatives';
  case 52
    str='incorrect constraint derivatives';
  case 61
    str='undefined function at the first feasible point';
  case 62
    str='undefined function at the initial point';
  case 63
    str='unable to proceed into undefined region';
  case 71
    str='terminated during function evaluation';
  case 74
    str='terminated from monitor routine';
  case 81
    str='work arrays must have at least 500 elements';
  case 82
    str='not enough character storage';
  case 83
    str='not enough integer storage';
  case 84
    str='not enough real storage';
  case 91
    str='invalid input argument';
  case 92
    str='basis file dimensions do not match this problem';
  case 141
    str='wrong number of basic variables';
  case 142
    str='error in basis package';
  otherwise
    str='some other error';
end

