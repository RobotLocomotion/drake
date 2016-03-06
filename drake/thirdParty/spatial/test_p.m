% test the correctness of the planar-vector dynamics routines FDabp, FDcrbp
% and IDp by checking that FDabp and FDcrbp are inverses of IDp, and by
% checking that HandCp gives the same result as HandC.

% step 1: create planar-vector and spatial-vector models of a complicated
% kinematic tree.

tree_p = autoTreep( 12, 1.5, 0.95 );	% planar-vector version
tree_s = autoTree( 12, 1.5, 0, 0.95 );	% spatial-vector version

% step 2: choose random initial conditions

q   = pi * (2*rand(12,1) - 1);
qd  = 2*rand(12,1) - 1;
qdd = 2*rand(12,1) - 1;

% step 3: use IDp to calculate the force required to produce qdd; then use
% both FDabp and FDcrbp to calculate the acceleration that this force
% produces.

tau = IDp( tree_p, q, qd, qdd );
qdd_ab = FDabp( tree_p, q, qd, tau );
qdd_crb = FDcrbp( tree_p, q, qd, tau );

% step 4: compare results.  In theory, we should have qdd_ab==qdd and
% qdd_crb==qdd.  However, rounding errors will make them slightly
% different.  Expect rounding errors in the vicinity of 1e-14 on this
% test.

(qdd_ab-qdd)'
(qdd_crb-qdd)'

% step 5: calculate H and C via both HandC and HandCp.  To make the test a
% little stronger, we shall use a custom value for gravity.

[Hs,Cs] = HandC( tree_s, q, qd, {}, [1;2;0] );
[Hp,Cp] = HandCp( tree_p, q, qd, {}, [1;2] );

% step 7: compare results.  Most rounding errors will be exactly zero in
% this test.  Nonzero errors will range from approximately 1e-17 to 1e-15.

(Cs-Cp)'
Hs-Hp
