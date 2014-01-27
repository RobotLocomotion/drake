function ret = spatialAccelerations(obj, kinsol, q, v, vd)
% SPATIALACCELERATIONS Computes the spatial accelerations (time derivatives
% of twists) of all bodies in the RigidBodyManipulator.
% @param kinsol solution structure obtained from doKinematics
% @retval twistdot cell array containing spatial accelerations of all rigid
% bodies with respect to the world

world = 1;


nBodies = length(obj.body);
ret = cell(nBodies, 1);
for i = 1 : nBodies
  if i == world
    ret{i} = zeros(6, 1);
  else
    body = obj.body(i);
    
    qBody = q(body.dofnum);
    vBody = v(body.dofnum);
    vdBody = vd(body.dofnum);
    
    predecessor = body.parent;
    
    predecessorAccel = obj.transformSpatialAcceleration(ret{predecessor}, kinsol, world, predecessor, predecessor, i);
    jointAccel = motionSubspace(body, qBody) * vdBody; % + motionSubspaceDot(body, qBody, vBody) * vBody;
    ret{i} = predecessorAccel + jointAccel;
  end
end

end
