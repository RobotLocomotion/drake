% test the correctness of the floating-base forward and inverse dynamics
% functions by checking that IDf is indeed the inverse of FDf.

% step 1: create a floating-base kinematic tree, and adjust some of the
% pitches so that it contains revolute, prismatic and helical joints.

NRJ = 8;				% number of real joints
tree = floatbase( autoTree( NRJ+1, 2, 1, 0.9 ) );
tree.pitch(7) = 0.1;			% 1st real joint
tree.pitch(8) = inf;			% 2nd real joint

% step 2: select random initial conditions.

q   = pi * (2*rand(NRJ,1) - 1);
qd  = 2*rand(NRJ,1) - 1;
qdd = 2*rand(NRJ,1) - 1;
X = Xrotz(2*pi*rand(1)) * Xroty(2*pi*rand(1)) * Xrotx(2*pi*rand(1)) * ...
    Xtrans(2*rand(3,1)-1);
v = 2*rand(6,1) - 1;

% step 3: use IDf to calculate the force required to produce qdd, and the
% resulting floating-base acceleration.  Then use FDf to calculate the
% accelerations produced by this force.

[a1,tau1] = IDf( tree, X, v, q, qd, qdd );
[a2,qdd2] = FDf( tree, X, v, q, qd, tau1 );

% step 4: compare results.  In theory, we should have a1==a2 and
% qdd==qdd2.  However, rounding errors will make them slightly different.
% Expect rounding errors in the vicinity of 1e-13 to 1e-14 on this test.

(a1-a2)'
(qdd-qdd2)'
