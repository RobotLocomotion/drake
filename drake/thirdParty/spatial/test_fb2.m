% test the correctness of the floating-base forward and inverse dynamics
% functions by checking them against data prepared by FDab.

% step 1: create a floating-base kinematic tree, and adjust some of the
% pitches so that it contains revolute, prismatic and helical joints.

NRJ = 8;				% number of real joints
tree = floatbase( autoTree( NRJ+1, 2, 1, 0.9 ) );
tree.pitch(7) = 0.1;			% 1st real joint
tree.pitch(8) = inf;			% 2nd real joint

% step 2: select random initial position, velocity and force conditions,
% and calculate the corresponding joint accelerations using FDab

q   = pi * (2*rand(NRJ+6,1) - 1);
qd  = 2*rand(NRJ+6,1) - 1;
tau = [0;0;0;0;0;0;2*rand(NRJ,1)-1];
qdd = FDab( tree, q, qd, tau );

% step 3: calculate the position, velocity and acceleration of the floating
% base using the kinematics function fbKin.

[X,v,a] = fbKin( q, qd, qdd );

% step 4: call IDf and FDf.

[a1,tau1] = IDf( tree, X, v, q(7:NRJ+6), qd(7:NRJ+6), qdd(7:NRJ+6) );
[a2,qdd2] = FDf( tree, X, v, q(7:NRJ+6), qd(7:NRJ+6), tau(7:NRJ+6) );

% step 5: compare results.  In theory, we should have a1==a, a2==a,
% tau1==tau(7:NRJ+6) and qdd2==qdd(7:NRJ+6).  However rounding errors will
% make them slightly different. Expect rounding errors in the vicinity of
% 1e-13 to 1e-14 on this test, and occasionally significantly larger
% errors.

[ a1-a, a2-a ]
[ tau1-tau(7:NRJ+6), qdd2-qdd(7:NRJ+6) ]
