function ret = spatialAccelerations(obj, kinsol, vd, root_accel)
% Computes the spatial accelerations (time derivatives of twists) of all
% bodies in the RigidBodyManipulator, expressed in world
%
% @param kinsol solution structure obtained from doKinematics
% @param vd joint acceleration vector
% @param root_accel spatial acceleration of root body @default zeros(6, 1)
%
% @retval ret cell array containing spatial accelerations of all rigid
% bodies with respect to the world, expressed in world

if nargin < 4
  root_accel = zeros(6, 1);
end

world = 1;
nbodies = length(obj.body);
ret = cell(nbodies, 1);
ret{world} = root_accel;
for i = 2 : nbodies
  body = obj.body(i);
  
  qbody = kinsol.q(body.position_num);
  vbody = kinsol.v(body.velocity_num);
  vdbody = vd(body.velocity_num);
  
  parent = body.parent;
  
  parent_accel = ret{parent};
  joint_accel_in_body = motionSubspace(body, qbody) * vdbody + motionSubspaceDotTimesV(body, qbody, vbody);
  joint_accel_in_base = transformSpatialAcceleration(kinsol, parent, i, i, world, joint_accel_in_body);
  ret{i} = parent_accel + joint_accel_in_base;
end

end
