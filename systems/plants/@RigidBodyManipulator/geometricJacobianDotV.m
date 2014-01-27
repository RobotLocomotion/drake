function jDotV = geometricJacobianDotV(obj, kinsol, base, endEffector, expressedIn)
% GEOMETRICJACOBIANDOTV computes the 'convective term' d/dt(J) * v, where J
% is a geometric Jacobian and v is the vector of joint velocities across
% the same joints that the geometric Jacobian spans
% @param kinsol solution structure obtained from doKinematics
% @param base base frame of the geometric Jacobian
% @param endEffector end effector frame of the geometric Jacobian
% @param expressedIn frame in which the geometric Jacobian is expressed
% @retval d/dt(J) * v

% Implementation makes use of the fact that
% d/dt(twist) = J * vdot + Jdot * v
% if we set vdot to zero, then Jdot * v can be found using any method to
% find the relative spatial acceleration between base and endEffector,
% expressed in expressedIn
% The relative spatial acceleration is the sum of the spatial accelerations
% across the individual joints (with vdot set to zero), transformed to
% expressedIn frame before addition.

[bodyPath, ~, ~] = obj.findKinematicPath(base, endEffector);

jDotV = zeros(6, 1);
for i = 2 : length(bodyPath)
  predecessor = bodyPath(i - 1);
  successor = bodyPath(i);
  vBody = kinsol.qd(obj.body(successor).dofnum);
  zeroJointAccel = zeros(6, 1); %motionSubspaceDot(body, qBody, vBody) * vBody; % spatial acceleration across joint when vdot across the joint is zero
  jDotV = jDotV + obj.transformSpatialAcceleration(zeroJointAccel, kinsol, predecessor, successor, predecessor, expressedIn);
end

end