function ret = computeSpatialAccelerations(obj, kinsol, vd, root_accel)
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
  
  qBody = kinsol.q(body.position_num);
  vBody = kinsol.v(body.velocity_num);
  vdBody = vd(body.velocity_num);
  
  predecessor = body.parent;
  
  predecessorAccel = ret{predecessor};
  jointAccelInBody = motionSubspace(body, qBody) * vdBody + motionSubspaceDotV(body, qBody, vBody);
  jointAccelInBase = transformSpatialAcceleration(kinsol, predecessor, i, i, world, jointAccelInBody);
  ret{i} = predecessorAccel + jointAccelInBase;
end

end
