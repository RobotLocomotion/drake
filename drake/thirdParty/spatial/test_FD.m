% test the correctness of FDab, FDcrb and ID by checking that FDab and
% FDcrb are inverses of ID.

% step 1: create a complicated kinematic tree, and adjust some of the
% pitches so that it contains helical and prismatic as well as revolute
% joints.

tree = autoTree( 12, 1.5, 1, 0.95 );
tree.pitch(3) = 0.1;
tree.pitch(5) = inf;
tree.pitch(7) = -0.1;
tree.pitch(9) = inf;

% step 2: choose random initial conditions

q   = pi * (2*rand(12,1) - 1);
qd  = 2*rand(12,1) - 1;
qdd = 2*rand(12,1) - 1;

% step 3: use ID to calculate the force required to produce qdd; then use
% both FDab and FDcrb to calculate the acceleration that this force
% produces.

tau = ID( tree, q, qd, qdd );
qdd_ab = FDab( tree, q, qd, tau );
qdd_crb = FDcrb( tree, q, qd, tau );

% step 4: compare results.  In theory, we should have qdd_ab==qdd and
% qdd_crb==qdd.  However, rounding errors will make them slightly
% different.  Expect rounding errors in the vicinity of 1e-14 on this
% test.

(qdd_ab-qdd)'
(qdd_crb-qdd)'
