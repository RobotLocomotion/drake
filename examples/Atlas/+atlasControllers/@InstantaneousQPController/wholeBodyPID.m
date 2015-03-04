function qddot_des = wholeBodyPID(obj, t, q, qd, q_des, params)
  % Run PID control on the whole body state. 
  % @param t time (s)
  % @param q, qd the current robot posture and velocity vectors
  % @param q_des the desired robot posture
  % @param params the whole_body field of an atlasParams.* object.
  if isnan(obj.q_integrator_data.t_prev)
    obj.q_integrator_data.t_prev = t;
  end
  dt = t - obj.q_integrator_data.t_prev;
  obj.q_integrator_data.t_prev = t;
  new_int_state = params.integrator.eta * obj.q_integrator_data.state + params.integrator.gains .* (q_des - q) * dt;

  if any(new_int_state)
    [joint_limits_min, joint_limits_max] = r.getJointLimits();
    new_int_state = max(-params.integrator.clamps, min(params.integrator.clamps, new_int_state));
    q_des = q_des + new_int_state;
    q_des = max(joint_limits_min - params.integrator.clamps,...
                min(joint_limits_max + params.integrator.clamps, q_des)); % allow it to go delta above and below jlims
  end
  obj.q_integrator_data.state = max(-params.integrator.clamps, min(params.integrator.clamps, new_int_state));

  err_q = [q_des(1:3) - q(1:3); angleDiff(q(4:end), q_des(4:end))];
  qddot_des = params.Kp .* err_q - params.Kd .* qd;
  qddot_des = max(params.qdd_bounds.min,...
                  min(params.qdd_bounds.max, qddot_des));
end