classdef Base 
  % Metaclass for storing gains for the Atlas controller and other parameters which
  % are fixed at compile time for a particular type of plan (walking, manipulating,
  % standing, recovery, etc.) but which can be switched out for a different set when
  % the plan type changes. 
  properties
    body_motion
    whole_body
    vref_integrator
    hardware
    joint_soft_limits
    W_kdot = zeros(3);
    Kp_ang = 0;
    w_slack = 0.05;
    slack_limit = 100;
    w_grf = 0.0;
    Kp_accel = 1.0;
    contact_threshold = 0.002;
    min_knee_angle = 0.7;
    use_center_of_mass_observer = false;
    center_of_mass_observer_gain;
  end

  methods 
    function obj = Base(r)
      typecheck(r, 'Atlas');
      nbod = r.getManipulator().getNumBodies();
      obj.body_motion = struct('Kp', mat2cell(12 * ones(6, nbod), 6, ones(1, nbod)),...
                               'damping_ratio', 0.7,...
                               'accel_bounds', struct('min', [-100;-100;-100;-50;-50;-50],...
                                                      'max', [100;100;100;50;50;50]),...
                               'weight', num2cell(zeros(1, nbod)),...
                               'Kd', mat2cell(zeros(6, nbod), 6, ones(1, nbod)));
      obj.body_motion(r.findLinkId('pelvis')).weight = 0.01;
      obj.body_motion(r.foot_body_id.right).weight = 0.01;
      obj.body_motion(r.findLinkId('pelvis')).Kp = [150; 150; 150; 200; 200; 200];
      obj.body_motion(r.findLinkId('pelvis')).damping_ratio = 0.6;
      obj.body_motion(r.foot_body_id.left).weight = 0.01;

      obj.whole_body = struct('Kp', 150*ones(r.getNumPositions(), 1),...
                              'damping_ratio', 0.6,...
                              'Kd', zeros(r.getNumPositions(), 1),...
                              'w_qdd', 1e-3 * ones(r.getNumVelocities(), 1),...
                              'integrator', struct('gains', zeros(r.getNumPositions(), 1),...
                                                   'clamps', zeros(r.getNumPositions(), 1),...
                                                   'eta', 0.0),...
                              'qdd_bounds', struct('min', -100*ones(r.getNumVelocities(), 1),...
                                                   'max', 100*ones(r.getNumVelocities(), 1)));
      obj.whole_body.w_qdd(r.findPositionIndices('base')) = 0;
      
      obj.vref_integrator = struct('zero_ankles_on_contact', 0,...
                                      'eta', 0.001,...
                                      'delta_max', 10.0);
      obj.whole_body.w_qdd(r.findPositionIndices('back_bkx')) = 0.1;
      obj.whole_body.Kp(r.findPositionIndices('back_bkx')) = 50;

      % Soft joint limits enforced by a sigmoid weight on q(i) - q_des(i)
      % The sigmoid is defined as:
      % w_lb = weight / (1 + exp(-k_logistic * (lb - q(i))))
      % w_ub = weight / (1 + exp(-k_logistic * (q(i) - ub)))
      % w_qdd = max(w_lb, w_lb)
      % where w_qdd is the cost weight of qdd_des, defined as:
      % qdd_des(i) = kp * (q_des(i) - q(i)) - kd * qd(i)
      % 
      % If disable_when_body_in_support = i != 0, then the joint soft limits will be disabled when body i is in support
      obj.joint_soft_limits = struct('enabled', num2cell(false(1, r.getNumPositions())),...
                                     'lb', num2cell(-inf(1, r.getNumPositions())),...
                                     'ub', num2cell(inf(1, r.getNumPositions())),...
                                     'kp', num2cell(repmat(150, 1, r.getNumPositions())),...
                                     'damping_ratio', num2cell(repmat(0.6, 1, r.getNumPositions())),...
                                     'kd', num2cell(zeros(1, r.getNumPositions())),... % Set when we call updateKd
                                     'weight', num2cell(repmat(1e-5, 1, r.getNumPositions())),...
                                     'disable_when_body_in_support', num2cell(zeros(1, r.getNumPositions)),...
                                     'k_logistic', num2cell(repmat(20, 1, r.getNumPositions())));
      obj.joint_soft_limits(r.findPositionIndices('r_leg_kny')).enabled = true;
      obj.joint_soft_limits(r.findPositionIndices('r_leg_kny')).lb = 0.5;
      obj.joint_soft_limits(r.findPositionIndices('r_leg_kny')).disable_when_body_in_support = r.foot_body_id.right;
      obj.joint_soft_limits(r.findPositionIndices('l_leg_kny')).enabled = true;
      obj.joint_soft_limits(r.findPositionIndices('l_leg_kny')).lb = 0.5;
      obj.joint_soft_limits(r.findPositionIndices('l_leg_kny')).disable_when_body_in_support = r.foot_body_id.left;

      nu = r.getNumInputs();
      obj.hardware = struct('gains', struct(...
                                      'k_f_p', zeros(nu,1),...
                                      'k_q_p', zeros(nu,1),...
                                      'k_q_i', zeros(nu,1),...
                                      'k_qd_p', zeros(nu,1),...
                                      'ff_qd', zeros(nu,1),...
                                      'ff_f_d', zeros(nu,1),...
                                      'ff_const', zeros(nu,1),...
                                      'ff_qd_d', zeros(nu,1)),...
                            'joint_is_force_controlled', ones(r.getNumInputs(), 1),...
                            'joint_is_position_controlled', zeros(r.getNumInputs(), 1));

      obj = obj.updateKd();
      
      l_zmp = 10;
      l_com = 2 * sqrt(l_zmp); % lower-right elements should be ~ 2 sqrt( upper-left elements) for critically damped response
      obj.center_of_mass_observer_gain = diag([l_zmp l_zmp l_com l_com]);
    end

    function obj = updateKd(obj)
      % Recompute derivative gains based on provided damping ratios
      obj.whole_body.Kd = getDampingGain(obj.whole_body.Kp, obj.whole_body.damping_ratio);
      for j = 1:length(obj.body_motion)
        obj.body_motion(j).Kd = getDampingGain(obj.body_motion(j).Kp, obj.body_motion(j).damping_ratio);
      end
      for j = 1:length(obj.joint_soft_limits)
        obj.joint_soft_limits(j).kd = getDampingGain(obj.joint_soft_limits(j).kp, obj.joint_soft_limits(j).damping_ratio);
      end
    end
  end
end
