function testRBTForwardKin_gradient()
r = dragon.RigidBodyTree(fullfile(getDrakePath(), 'examples/Pendulum/Pendulum.urdf'));

valuecheck(r.num_positions, 7);
valuecheck(r.num_velocities, 7);

q = dragon.newAutoDiff(zeros(7,1), eye(7));
v = dragon.newAutoDiff(zeros(7,1), zeros(7, 7));
kinsol = r.doKinematics(q, v);
point = ones(3,1);
p = r.forwardKin(kinsol, point, 2, 0, 0);
valuecheck(p.value(), ones(3,1));
valuecheck(p.derivatives(), [1, 0, 0, 0, 1, -1, 1;
                             0, 1, 0, -1, 0, 1, 0;
                             0, 0, 1, 1, -1, 0, -1]);
end
