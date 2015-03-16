function testPositionConstraints

r = RigidBodyManipulator('examples/Atlas/urdf/robotiq.urdf');
r_new = RigidBodyManipulator('examples/Atlas/urdf/robotiq.urdf', struct('use_new_kinsol', true));
x0 = r.getInitialState;
q0 = x0(1:r.num_positions);
v0 = x0(r.num_positions + (1:r.num_velocities));
%[H C B] = r.manipulatorDynamics(q0, v0)

disp('test starting')
kinsol = r.doKinematics(q0);
[phiC,normal,~,~,~,~,~,mu,n,D] = r.contactConstraints(kinsol,true);

kinsol = r_new.doKinematics(q0);
[phiC_new,normal_new,~,~,~,~,~,mu,n_new,D_new] = r_new.contactConstraints(kinsol,true);

valuecheck(phiC, phiC_new);
valuecheck(normal, normal_new);
valuecheck(n, n_new);
for i = 1:length(D)
valuecheck(D{i}, D_new{i});
end
setupLCPmex(r_new.mex_model_ptr, q0, v0);
end

