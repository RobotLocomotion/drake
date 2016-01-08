function testRBTIK_posture_constraint()
checkDependency('cpp_bindings');
checkDependency('snopt');

r = rbtree.RigidBodyTree(fullfile(getDrakePath(), 'examples/Pendulum/Pendulum.urdf'));
q = -0.9;
posture_constraint = ik.PostureConstraint(r);
posture_constraint.setJointLimits(int32(6), q, q);
% Choose a seed configuration and a nominal configuration (at 0)
q_seed = [zeros(6,1); 0.8147];
q_nom = [zeros(6,1); 0.0];

options = ik.IKoptions(r);
results = ik.inverseKinSimple(r,...
                              q_seed,...
                              q_nom,...
                              {posture_constraint},...
                              options);
q_sol = results.q_sol;
valuecheck(q_sol(7), q, 1e-9);

end