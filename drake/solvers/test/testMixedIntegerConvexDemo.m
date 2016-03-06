function testMixedIntegerConvexDemo
% Demonstrate a few features of the new mixed-integer convex program interface.

% Let's say we want to perform a simple mixed-integer quadratic optimization:
% minimize ||(x - xgoal)||^2
% subject to A1x <= b1 OR A2x <= b2
%
% we can construct that problem very easily using the symbolic version of the 
% MICP:
[xgoal, A1, b1, A2, b2, pts1, pts2] = getRandomProblem();

t0 = tic();
p = MixedIntegerConvexProgram(true);
% add our continuous variable
p = p.addVariable('x', 'C', [2, 1], -1, 1);
% add a binary variable to indicate which convex region x lies in. Our convention
% is that region(i) implies that Ai * x <= bi
p = p.addVariable('region', 'B', [2, 1], 0, 1);

% Now we can add a cost with a symbolic expression
x = p.vars.x.symb;
p = p.addSymbolicCost((x - xgoal)' * (x - xgoal));

% and the symbolic constraints
region = p.vars.region.symb;
p = p.addSymbolicConstraints([implies(region(1), A1  * x <= b1), ...
                              implies(region(2), A2 * x <= b2),...
                              sum(region) == 1]);

% solve the model
[p, solvertime] = p.solve();
fprintf(1, 'symbolic problem overhead: %fs\n', toc(t0) - solvertime);


% and get the result
xstar = p.vars.x.value

figure(1);
clf
hold on
k = convhull(pts1(1,:), pts1(2,:));
patch(pts1(1,k), pts1(2,k), 'k', 'FaceAlpha', 0.2)
k = convhull(pts2(1,:), pts2(2,:));
patch(pts2(1,k), pts2(2,k), 'k', 'FaceAlpha', 0.2)
plot(xgoal(1), xgoal(2), 'ro', 'MarkerSize', 12);
plot(xstar(1), xstar(2), 'g*', 'MarkerSize', 10);

% However, the setup time for symbolic optimizations in Yalmip can get very long, 
% and often requires more time than is needed to solve the resulting model. We 
% can eliminate that setup time by constructing the constraint and objective 
% matrices directly. Let's construct the same problem, but with symbolic expressions
% turned off:

t1 = tic();
p = MixedIntegerConvexProgram(false);
p = p.addVariable('x', 'C', [2, 1], -1, 1);
p = p.addVariable('region', 'B', [2, 1], 0, 1);

% Dealing with indices in optimizations is a pain because we think about lots of separate
% decision variables with various shapes, but the optimizer just sees one big vector of
% decision variables. To make this a little easier, the MICP interface provides a field 'i'
% for every variable, which gives the linear indices of that decision variable in the correct
% shape for that variable. This is easiest to see with an example. Let's construct our 
% quadratic cost matrix:
Q = zeros(p.nv, p.nv); % p.nv is the total number of decision variables
Q(p.vars.x.i(1:2), p.vars.x.i(1:2)) = eye(2); % p.vars.x.i(1) is the index into the vector of decision variables corresponding to the first element of x
c = zeros(p.nv, 1);
c(p.vars.x.i(1:2)) = - 2 * xgoal;
objcon = xgoal' * xgoal;

p = p.addCost(Q, c, objcon);

% now we add our constraints. To express the constraint that region(i) implies that Ai * x <= b
% we will use a "big M" formulation, in which we replace our constraint with:
% Ai * x <= b + M(1-region(i))
% where M is a sufficiently large constant. You can learn more about this form here:
% http://users.isy.liu.se/johanl/yalmip/pmwiki.php?n=Tutorials.Big-MAndConvexHulls
m = size(A1, 1);
A = zeros(m, p.nv);
M = 10;
A(:,p.vars.x.i(1:2)) = A1;
A(:,p.vars.region.i(1)) = M;
p = p.addLinearConstraints(A, b1 + M, [], []);

m = size(A2, 1);
A = zeros(m, p.nv);
A(:,p.vars.x.i(1:2)) = A2;
A(:,p.vars.region.i(2)) = M;
p = p.addLinearConstraints(A, b2 + M, [], []);

% finally, we constrain that sum(region) == 1
Aeq = zeros(1, p.nv);
Aeq(1, p.vars.region.i) = 1;
beq = 1;
p = p.addLinearConstraints([], [], Aeq, beq);

[p, solvertime] = p.solve();
fprintf(1, 'non-symbolic problem overhead: %fs\n', toc(t1) - solvertime);
xstar_non_symb = p.vars.x.value
valuecheck(xstar_non_symb, xstar);


% However, compiling constraints and objectives down to raw matrix manipulation
% is difficult and error-prone, so the MixedIntegerConvexProgram makes it easy
% to mix symbolic and non-symbolic constraints as a way to more smoothly 
% transition from one form to the other. For example, let's set up a third problem
% with symbolics enabled:
p = MixedIntegerConvexProgram(true);
p = p.addVariable('x', 'C', [2, 1], -1, 1);
p = p.addVariable('region', 'B', [2, 1], 0, 1);

% we'll add our cost using a matrix form:
Q = zeros(p.nv, p.nv);
Q(p.vars.x.i(1:2), p.vars.x.i(1:2)) = eye(2);
c = zeros(p.nv, 1);
c(p.vars.x.i(1:2)) = - 2 * xgoal;
objcon = xgoal' * xgoal;
p = p.addCost(Q, c, objcon);

% but add our constraints symbolically:
x = p.vars.x.symb;
region = p.vars.region.symb;
p = p.addSymbolicConstraints([implies(region(1), A1  * x <= b1), ...
                              implies(region(2), A2 * x <= b2),...
                              sum(region) == 1]);
[p, solvertime] = p.solve();
xstar_mixed = p.vars.x.value
valuecheck(xstar_mixed, xstar);



end

function [xgoal, A1, b1, A2, b2, pts1, pts2] = getRandomProblem()
  xgoal = rand(2,1);
  pts1 = [rand(2,4)];
  pts2 = [rand(2,4)];
  [A1, b1] = poly2lincon(pts1(1,:), pts1(2,:));
  [A2, b2] = poly2lincon(pts2(1,:), pts2(2,:));
  % normalize the linear constraints to make our formulation numerically nicer:
  for i = 1:size(A1, 1)
    n = norm(A1(i,:));
    A1(i,:) = A1(i,:) / n;
    b1(i) = b1(i) / n;
  end
  for i = 1:size(A2, 1)
    n = norm(A2(i,:));
    A2(i,:) = A2(i,:) / n;
    b2(i) = b2(i) / n;
  end
end

