function testRBTTransformPoints_value()
checkDependency('cpp_bindings');

r = rbtree.RigidBodyTree(fullfile(getDrakePath(), 'examples/Pendulum/Pendulum.urdf'));

valuecheck(r.num_positions, 7);
valuecheck(r.num_velocities, 7);
kinsol = r.doKinematics(zeros(7,1), zeros(7,1));
p = r.transformPoints(kinsol, zeros(3,1), 0, 1);
valuecheck(p, zeros(3,1));

end
