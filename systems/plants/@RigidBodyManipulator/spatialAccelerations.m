function ret = spatialAccelerations(obj, kinsol, q, v, vd)
% SPATIALACCELERATIONS Computes the spatial accelerations (time derivatives
% of twists) of all bodies in the RigidBodyManipulator
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
    
    predecessorAccel = spatialAccelerationChangeFrame(obj, ret{predecessor}, kinsol, world, predecessor, predecessor, i);
    jointAccel = motionSubspace(body, qBody) * vdBody; % + motionSubspaceDot(body, qBody, vBody) * vBody;
    ret{i} = predecessorAccel + jointAccel;
  end
end

end

function spatialAccel = spatialAccelerationChangeFrame(obj, spatialAccel, kinsol, base, body, oldExpressedIn, newExpressedIn)

twistOfBodyWrtBase = obj.relativeTwist(kinsol, base, body, oldExpressedIn);
twistOfOldWrtNew = obj.relativeTwist(kinsol, newExpressedIn, oldExpressedIn, oldExpressedIn);
transformFromOldToNew = kinsol.T{newExpressedIn} \ kinsol.T{oldExpressedIn};

spatialAccel = transformAdjoint(transformFromOldToNew) * (twistAdjoint(twistOfOldWrtNew) * twistOfBodyWrtBase + spatialAccel);

% this should be faster:
% omegaOfBodyWrtBase = twistOfBodyWrtBase(1 : 3);
% vOfBodyWrtBase = twistOfBodyWrtBase(4 : 6);
% 
% omegaOfOldWrtNew = twistOfOldWrtNew(1 : 3);
% vOfOldWrtNew = twistOfOldWrtNew(4 : 6);
% 
% omegadot = spatialAccel(1 : 3);
% vdot = spatialAccel(4 : 6);
% 
% oldToNewR = transformFromOldToNew(1 : 3, 1 : 3);
% oldToNewp = transformFromOldToNew(1 : 3, 4);
% 
% omegadotNew = oldToNewR * (cross(omegaOfOldWrtNew, omegaOfBodyWrtBase) + omegadot);
% vdotNew = cross(oldToNewp, omegadotNew) + oldToNewR * (cross(vOfOldWrtNew, omegaOfBodyWrtBase) + cross(omegaOfOldWrtNew, vOfBodyWrtBase) + vdot);
% 
% spatialAccel = [omegadotNew; vdotNew];

end

function adT = twistAdjoint(twist)
omega = twist(1 : 3);
v = twist(4 : 6);
adT = [vectorToSkewSymmetric(omega), zeros(3, 3);
       vectorToSkewSymmetric(v), vectorToSkewSymmetric(omega)];
end