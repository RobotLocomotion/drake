function v_ref = velocityReference(obj, t, q, qd, qdd, foot_contact_sensor, params)
  % Integrate expected accelerations to compute a feed-forward velocity reference. 
  % @param t time (s)
  % @param q, qd robot state
  % @param qdd accelerations computed by the QP
  % @param foot_contact_sensor 2x1 flags indicating whether contact is detected
  %                            for the [left; right] foot.
  % @param params the vref_integrator field of an atlasParams.* object.
  fc = struct('right', foot_contact_sensor(2) > 0.5,...
                        'left', foot_contact_sensor(1) > 0.5);

  if isnan(obj.vref_integrator_data.t_prev)
    obj.vref_integrator_data.t_prev = t;
  end
  dt = t - obj.vref_integrator_data.t_prev;
  obj.vref_integrator_data.t_prev = t;

  qd_int = obj.vref_integrator_data.state;
  qd_int = (1-params.eta)*qd_int + params.eta*qd + qdd*dt;

  if params.zero_ankles_on_contact && fc.left
    qd_int(obj.robot_property_cache.position_indices.l_leg_ak) = 0;
  end
  if params.zero_ankles_on_contact && fc.right
    qd_int(obj.robot_property_cache.position_indices.r_leg_ak) = 0;
  end

  fc_prev = obj.vref_integrator_data.fc_prev;
  if fc_prev.left ~= fc.left
    % contact state changed, reset integrated velocities
    qd_int(obj.robot_property_cache.position_indices.l_leg) = qd(obj.robot_property_cache.position_indices.l_leg);
  end
  if fc_prev.right ~= fc.right
    qd_int(obj.robot_property_cache.position_indices.r_leg) = qd(obj.robot_property_cache.position_indices.r_leg);
  end

  obj.vref_integrator_data.fc_prev = fc;
  obj.vref_integrator_data.state = qd_int;

  qd_err = qd_int - qd;

  % do not velocity control ankles when in contact
  if params.zero_ankles_on_contact && fc.left
    qd_err(obj.robot_property_cache.position_indices.l_leg_ak) = 0;
  end
  if params.zero_ankles_on_contact && fc.right
    qd_err(obj.robot_property_cache.position_indices.r_leg_ak) = 0;
  end

  delta_max = 1.0;
  v_ref = max(-delta_max,min(delta_max,qd_err(obj.robot_property_cache.actuated_indices)));
end