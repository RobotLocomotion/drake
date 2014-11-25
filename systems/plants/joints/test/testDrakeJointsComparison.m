function testDrakeJointsComparison()

if exist('testDrakeJointsmex','file')~=3
  error('Drake:MissingDependency', 'Cannot find testDrakeJointsmex. It may not have been compiled due to a missing dependency.');
end

data_in.prismatic.joint_axis = randn(3, 1);
data_in.prismatic.joint_axis = data_in.prismatic.joint_axis / norm(data_in.prismatic.joint_axis);
data_in.prismatic.q = randn;
data_in.prismatic.v = randn;

data_in.revolute.joint_axis = randn(3, 1);
data_in.revolute.joint_axis = data_in.revolute.joint_axis / norm(data_in.revolute.joint_axis);
data_in.revolute.q = randn;
data_in.revolute.v = randn;

data_in.helical.joint_axis = randn(3, 1);
data_in.helical.joint_axis = data_in.helical.joint_axis / norm(data_in.helical.joint_axis);
data_in.helical.pitch = randn;
data_in.helical.q = randn;
data_in.helical.v = randn;

data_in.rpy_floating.q = [randn(3, 1); uniformlyRandomRPY()];
data_in.rpy_floating.v = randn(6, 1);

data_in.quaternion_floating.q = [randn(3, 1); uniformlyRandomQuat()];
data_in.quaternion_floating.v = randn(6, 1);

data_out = testDrakeJointsmex(data_in);

data_in_matlab.prismatic_body.floating = 0;
data_in_matlab.prismatic_body.pitch = inf;
data_in_matlab.prismatic_body.joint_axis = data_in.prismatic.joint_axis;

data_in_matlab.revolute_body.floating = 0;
data_in_matlab.revolute_body.pitch = 0;
data_in_matlab.revolute_body.joint_axis = data_in.revolute.joint_axis;

data_in_matlab.helical_body.floating = 0;
data_in_matlab.helical_body.pitch = data_in.helical.pitch;
data_in_matlab.helical_body.joint_axis = data_in.helical.joint_axis;

data_in_matlab.rpy_floating_body.floating = 1;

data_in_matlab.quaternion_floating_body.floating = 2;

fields = fieldnames(data_in);
for i = 1 : length(fields)
  body = data_in_matlab.([fields{i} '_body']);
  data_out_i = data_out.(fields{i});
  data_in_i = data_in.(fields{i});
  
  q = data_in_i.q;
  v = data_in_i.v;
  
  joint_transform = jointTransform(body, q);
  valuecheck(data_out_i.joint_transform, joint_transform);
  
  [motion_subspace, dmotion_subspace] = motionSubspace(body, q);
  valuecheck(data_out_i.motion_subspace, motion_subspace);
  valuecheck(data_out_i.dmotion_subspace, dmotion_subspace);
  
  [motion_subspace_dot_times_v, dmotion_subspace_dot_times_vdq, dmotion_subspace_dot_times_vdv] = motionSubspaceDotTimesV(body, q, v);
  valuecheck(data_out_i.motion_subspace_dot_times_v, motion_subspace_dot_times_v);
  valuecheck(data_out_i.dmotion_subspace_dot_times_vdq, dmotion_subspace_dot_times_vdq);
  valuecheck(data_out_i.dmotion_subspace_dot_times_vdv, dmotion_subspace_dot_times_vdv);
  
  [qdot_to_v, dqdot_to_v] = jointQdot2v(body, q);
  valuecheck(data_out_i.qdot_to_v, qdot_to_v);
  valuecheck(data_out_i.dqdot_to_v, dqdot_to_v);
  
  [v_to_qdot, dv_to_qdot] = jointV2qdot(body, q);
  valuecheck(data_out_i.v_to_qdot, v_to_qdot);
  valuecheck(data_out_i.dv_to_qdot, dv_to_qdot);
  
end


end
