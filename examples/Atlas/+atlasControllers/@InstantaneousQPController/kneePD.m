function [w_qdd, qddot_des] = kneePD(obj, q, qd, w_qdd, qddot_des, toe_off, min_knee_angle)
  % Implement our toe-off logic for the knee. Not currently in use, but
  % kept here for eventual inclusion.
  kp = 40;
  kd = 4;
  r_knee_inds = obj.robot_property_cache.position_indices.r_leg_kny;
  l_knee_inds = obj.robot_property_cache.position_indices.l_leg_kny;
  if toe_off.right
    r_kny_qdd_des = kp*(min_knee_angle-q(r_knee_inds)) - kd*qd(r_knee_inds);
    qddot_des(r_knee_inds) = r_kny_qdd_des;
    w_qdd(r_knee_inds) = 1;
  elseif q(r_knee_inds) < min_knee_angle
    w_qdd(r_knee_inds) = 1e-4;
  end
  if toe_off.left
    l_kny_qdd_des = kp*(min_knee_angle-q(l_knee_inds)) - kd*qd(l_knee_inds);
    qddot_des(l_knee_inds) = l_kny_qdd_des;
    w_qdd(l_knee_inds) = 1;
  elseif q(l_knee_inds) < min_knee_angle
    w_qdd(l_knee_inds) = 1e-4;
  end
end
