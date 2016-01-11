function testRBTTransformPoints_gradient()
checkDependency('cpp_bindings');

r = rbtree.RigidBodyTree(fullfile(getDrakePath(), 'examples/Pendulum/Pendulum.urdf'));

valuecheck(r.num_positions, 7);
valuecheck(r.num_velocities, 7);

q = autodiffutils.toAutoDiff(zeros(7,1), eye(7));
v = autodiffutils.toAutoDiff(zeros(7,1), zeros(7, 7));
kinsol = r.doKinematics(q, v);
point = ones(3,1);
p = r.transformPoints(kinsol, point, 2, 0);
valuecheck(p.value(), ones(3,1));
valuecheck(p.derivatives(), [1, 0, 0, 0, 1, -1, 1;
                             0, 1, 0, -1, 0, 1, 0;
                             0, 0, 1, 1, -1, 0, -1]);
end
