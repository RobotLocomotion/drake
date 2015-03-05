function robotiq_springs
robot = TimeSteppingRigidBodyManipulator('robotiq_springs.urdf', 0.01);
v = robot.constructVisualizer;
v.inspector;
end

