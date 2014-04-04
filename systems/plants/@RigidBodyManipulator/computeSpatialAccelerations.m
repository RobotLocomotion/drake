function ret = computeSpatialAccelerations(obj, transforms, twists, q, v, vd, root_accel)
% COMPUTESPATIALACCELERATIONS Computes the spatial accelerations (time 
% derivatives of twists) of all bodies in the RigidBodyManipulator,
% expressed in world
% @param transforms homogeneous transforms from link to world (usually
% obtained from doKinematics as kinsol.T)
% @param twists twists of links with respect to world
% @retval twistdot cell array containing spatial accelerations of all rigid
% bodies with respect to the world, expressed in world

if nargin < 7
  root_accel = zeros(6, 1);
end

world = 1;
nBodies = length(obj.body);
ret = cell(nBodies, 1);
ret{world} = root_accel;
for i = 2 : nBodies
  body = obj.body(i);
  
  qBody = q(body.position_num);
  vBody = v(body.velocity_num);
  vdBody = vd(body.velocity_num);
  
  predecessor = body.parent;
  
  predecessorAccel = ret{predecessor};
  jointAccelInBody = motionSubspace(body, qBody) * vdBody + motionSubspaceDotV(body, qBody, vBody);
  jointAccelInBase = transformSpatialAcceleration(jointAccelInBody, transforms, twists, predecessor, i, i, world);
  ret{i} = predecessorAccel + jointAccelInBase;
  
  % implementation with spatial accelerations expressed in body frame:
  % predecessorAccel = transformSpatialAcceleration(ret{predecessor}, transforms, twists, world, predecessor, predecessor, i);
  % jointAccel = motionSubspace(body, qBody) * vdBody + motionSubspaceDotV(body, qBody, vBody);
  % ret{i} = predecessorAccel + jointAccel;
end

end
