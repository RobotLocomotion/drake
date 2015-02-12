function jDotV = geometricJacobianDotV(obj, kinsol, base, endEffector, expressedIn)
% GEOMETRICJACOBIANDOTV computes the 'convective term' d/dt(J) * v, where J
% is a geometric Jacobian and v is the vector of joint velocities across
% the same joints that the geometric Jacobian spans
% @param kinsol solution structure obtained from doKinematics
% @param twists twists of all links with respect to base, expressed in body
% frame
% @param base base frame of the geometric Jacobian
% @param endEffector end effector frame of the geometric Jacobian
% @param expressedIn frame in which the geometric Jacobian is expressed
% @retval d/dt(J) * v

% Implementation makes use of the fact that
% d/dt(twist) = J * vdot + Jdot * v
% If we set vdot to zero, then Jdot * v can be found using any algorithm
% that computes the relative spatial acceleration between base and
% endEffector, expressed in expressedIn.
% The relative spatial acceleration is the sum of the spatial accelerations
% across the individual joints (with vdot set to zero), transformed to
% expressedIn frame before addition.

if (kinsol.mex)
    error('Drake:RigidBodyManipulator:NoMex','This method has not been mexed yet.');
end

[~, jointPath, signs] = obj.findKinematicPath(base, endEffector);

jDotV = zeros(6, 1);
for i = 1 : length(jointPath)
  successor = jointPath(i);
  successorBody = obj.body(successor);
  predecessor = successorBody.parent;
  qBody = kinsol.q(successorBody.position_num);
  vBody = kinsol.qd(successorBody.velocity_num);
  zeroJointAccel = motionSubspaceDotTimesV(successorBody, qBody, vBody); % spatial acceleration across joint when vdot across the joint is zero
  jDotV = jDotV + signs(i) * transformSpatialAcceleration(kinsol, predecessor, successor, successor, expressedIn, zeroJointAccel);
end

end