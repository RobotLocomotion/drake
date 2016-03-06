function gloptipoly_constrained_test

checkDependency('spotless');

x = msspoly('x',3);
objective = -2*x(1)+x(2)-x(3);
ineq = [-(24-20*x(1)+9*x(2)-13*x(3)+4*x(1)^2-4*x(1)*x(2)+4*x(1)*x(3)+2*x(2)^2-2*x(2)*x(3)+2*x(3)^2); ...
  x(1)+x(2)+x(3)-4; ...
  3*x(2)+x(3)-6];

prog = PolynomialProgram(x,objective,ineq);
prog = prog.addConstraint(BoundingBoxConstraint([0;0;0],[2;inf;3]));

[x,objval,exitflag] = gloptipoly(prog)

% note: the doc says that this requires relaxation order = 4 before it
% can guarantee optimality.  should i put a loop into the code to run til
% it converges?