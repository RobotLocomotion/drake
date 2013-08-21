function [ptA,ptB,normal,distance,JA,JB,dJA,dJB] = pairwiseClosestPoints(obj,kinsol,bodyA_idx,bodyB_idx)

% Uses bullet to find the points of closest approach between bodies A and B
%
% @param kinsol  the output from doKinematics()
% @param bodyA_idx  numerical index of rigid body A or 
%     (less efficient:) a rigidbody object
% @param bodyB_idx  numerical index of rigid body B (or the rigidbody object)
%     if bodyB_idx is -1, compute collisions with entire world
% 
% @retval ptA the points (in world coordinates) on bodyA that are in
% collision
% @retval ptB the point (in world coordinates) on bodyB that are in
% collision
% @retval normal the contact normal vectors on body B in world coordinates
% @retval JA the jacobian of ptA
% @retval JB the jacobian of ptB
% @ingroup Collision

if ~isstruct(kinsol)  
  % treat input as contactPositions(obj,q)
  kinsol = doKinematics(obj,kinsol,nargout>5);
end

if (kinsol.mex ~= true) 
  error('need to call doKinematics using mex first');
end

if (isa(bodyA_idx,'RigidBody')) bodyA_idx = find(obj.body==bodyA_idx,1); end
if (isa(bodyB_idx,'RigidBody')) bodyB_idx = find(obj.body==bodyB_idx,1); end

[ptA,ptB,normal,distance] = collisionmex(obj.mex_model_ptr,4,bodyA_idx,bodyB_idx);

if isempty(ptA)
  JA=[]; JB=[];
  error('ptA should not be empty');
  return;
end

if (nargout>3)
  x = bodyKin(obj,kinsol,bodyA_idx,ptA);
  if (nargout>5)
    [~,JA,dJA] = forwardKin(obj,kinsol,bodyA_idx,x);
  else
    [~,JA] = forwardKin(obj,kinsol,bodyA_idx,x);
  end
end
if (nargout>4)
  x = bodyKin(obj,kinsol,bodyB_idx,ptB);
  if (nargout>6)
    [~,JB,dJB] = forwardKin(obj,kinsol,bodyB_idx,x);
  else
    [~,JB] = forwardKin(obj,kinsol,bodyB_idx,x);
  end
end
