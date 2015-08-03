% test the correctness of the hybrid dynamics function, HD, by comparing it
% with data prepared by ID.

% step 1: create a complicated kinematic tree, and adjust some of the
% pitches so that it contains helical and prismatic as well as revolute
% joints.

tree = autoTree( 12, 1.5, 1, 0.95 );
tree.pitch(3) = 0.1;
tree.pitch(5) = inf;
tree.pitch(7) = -0.1;
tree.pitch(9) = inf;

% step 2: choose random initial conditions, and compute corresponding force
% vector.

q   = pi * (2*rand(12,1) - 1);
qd  = 2*rand(12,1) - 1;
qdd = 2*rand(12,1) - 1;
tau = ID( tree, q, qd, qdd );

% step 3: choose a random forward-dynamics joint set, and call HD.

fd = rand(12,1) > 0.5;

[qdd_out,tau_out] = HD( tree, fd, q, qd, qdd, tau );

% step 4: compare results.  In theory, we should have qdd_out==qdd and
% tau_out==tau.  However, rounding errors will make the calculated values
% (but not the copied values) slightly different.  Thus, for each joint i
% such that fd(i)==1, we would expect a rounding error in qdd_out(i); and
% for each joint i such that fd(i)==0, we would expect a rounding error in
% tau_out(i).  Expect rounding errors in the vicinity of 1e-14 on this test.

[ qdd_out-qdd, tau_out-tau ]
