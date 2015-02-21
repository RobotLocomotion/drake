function [com_Jacobian_dot_times_v, dcom_Jacobian_dot_times_v] = ...
  centerOfMassJacobianDotTimesV(obj,kinsol,robotnum)
% computes the quantity Jd * v == d(dcom/dq) * qd, where J is the
% second output of centerOfMassJacobianV.
%
% Algorithm: let J be the matrix such that J * v = comd. Take derivative:
% Jd * v + J * vd = comdd, so Jd * v = comdd when vd = 0 --> can use an
% algorithm that computes the COM acceleration with vd set to zero. COM
% acceleration is robot linear momentum rate of change divided by total
% mass. Robot linear momentum rate of change is sum of linear momentum
% rates of change of individual links. Individual link momentum rates of
% change are link COM acceleration times link mass.
%
% @param kinsol solution structure obtained from doKinematics
% @param robotnum an int array. Jdot * v for the bodies that belong to
% robot(robotnum) is computed. Default is 1.
%
% @retval com_Jacobian_dot_times_v Jdot * v == d(dcom/dq) * qdot
% @retval dcom_Jacobian_dot_times_v gradient with respect to q

compute_gradients = nargout > 1;
if(nargin < 3)
  robotnum = 1;
end

if (kinsol.mex)
  if robotnum ~= 1
    error('not yet implemented');
  end
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  if compute_gradients
    [com_Jacobian_dot_times_v, dcom_Jacobian_dot_times_v] = centerOfMassJacobianDotTimesVmex(obj.mex_model_ptr);
  else
    [com_Jacobian_dot_times_v] = centerOfMassJacobianDotTimesVmex(obj.mex_model_ptr);
  end
else
  com_Jacobian_dot_times_v = zeros(3, 1) * kinsol.q(1);
  if compute_gradients
    nq = obj.getNumPositions();
    dcom_Jacobian_dot_times_v = zeros(numel(com_Jacobian_dot_times_v), nq);
  end
  
  total_mass = 0;
  for i = 2 : obj.getNumBodies()
    body = obj.body(i);
    if any(body.robotnum == robotnum)
      body_mass = body.mass;
      body_com = body.com;
      
      if compute_gradients
        [body_comdd, dbody_comdd] = pointAcceleration(kinsol, i, body_com);
        dbody_linear_momentum_dot = body_mass * dbody_comdd;
        dcom_Jacobian_dot_times_v = dcom_Jacobian_dot_times_v + dbody_linear_momentum_dot;
      else
        body_comdd = pointAcceleration(kinsol, i, body_com);
      end
      body_linear_momentum_dot = body_mass * body_comdd;
      com_Jacobian_dot_times_v = com_Jacobian_dot_times_v + body_linear_momentum_dot;
      
      total_mass = total_mass + body_mass;
    end
  end
  com_Jacobian_dot_times_v = com_Jacobian_dot_times_v / total_mass;
  if compute_gradients
    dcom_Jacobian_dot_times_v = dcom_Jacobian_dot_times_v / total_mass;
  end
end
end

function [point_accel, dpoint_accel] = pointAcceleration(kinsol, body_index, point)
% computes the acceleration of a point fixed in body frame with respect to
% the world, assuming zero joint accelerations (i.e. accelerations are due
% to coriolis/centripetal effects only)

compute_gradient = nargout > 1;

twist = kinsol.twists{body_index};
omega = twist(1:3);
v = twist(4:6);

spatial_accel = kinsol.JdotV{body_index};
omegad = spatial_accel(1:3);
vd = spatial_accel(4:6);

T = kinsol.T{body_index};
R = T(1:3,1:3);
p = T(1:3,4);
point_base = R * point + p;

point_vel = cross(omega, point_base) + v;
point_accel = cross(omegad, point_base) + vd + cross(omega, point_vel);

if compute_gradient
  dtwist = kinsol.dtwistsdq{body_index};
  domega = dtwist(1:3, :);
  dv = dtwist(4:6, :);
  
  dspatial_accel = kinsol.dJdotVdq{body_index};
  domegad = dspatial_accel(1:3, :);
  dvd = dspatial_accel(4:6, :);
  
  dT = kinsol.dTdq{body_index};
  dR = getSubMatrixGradient(dT, 1:3, 1:3, size(T));
  dp = getSubMatrixGradient(dT, 1:3, 4, size(T));
  dpoint_base = matGradMult(dR, point) + dp;
  
  dpoint_vel = dcross(omega, point_base, domega, dpoint_base) + dv;
  dpoint_accel = dcross(omegad, point_base, domegad, dpoint_base) + ...
    dvd + dcross(omega, point_vel, domega, dpoint_vel);
end
end
