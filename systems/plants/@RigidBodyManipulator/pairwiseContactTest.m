function [ptsA,ptsB,JA,JB] = pairwiseContactTest(obj,kinsol,body_indA,body_indB)

% Uses bullet to perform collision detection between rigid body A and rigid body B
%
% @param kinsol  the output from doKinematics()
% @param body_indA  numerical index of rigid body A or 
%     (less efficient:) a rigidbody object
% @param body_indB  numerical index of rigid body B (or the rigidbody object)
%
% @retval ptsA the points (in world coordinates) on bodyA that are in
% collision
% @retval ptsB the point (in world coordinates) on bodyB that are in
% collision
% @retval JA the jacobian of ptsA
% @retval JB the jacobian of ptsB

if (kinsol.mex ~= true) 
  error('need to call doKinematics using mex first');
end

if (isa(body_indA,'RigidBody')) body_indA = find(obj.body==body_indA,1); end
if (isa(body_indB,'RigidBody')) body_indB = find(obj.body==body_indB,1); end

[ptsA,ptsB] = collisionmex(obj.mex_model_ptr.getData,1,body_indA,body_indB);

if (nargin>2)
  x = bodyKin(obj,kinsol,body_indA,ptsA);
  [~,JA] = forwardKin(obj,body_indA,x);
end
if (nargin>3)
  x = bodyKin(obj,kinsol,body_indB,ptsB);
  [~,JB] = forwardKin(obj,body_indB,x);
end
