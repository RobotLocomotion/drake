function testDrakeJointsComparison()
data_in.prismatic.joint_axis = randn(3, 1);
data_in.revolute.joint_axis = randn(3, 1);
data_in.helical.joint_axis = randn(3, 1);
data_in.helical.pitch = randn;

testDrakeJointsmex(data_in);

end

