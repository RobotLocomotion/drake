classdef Base 
  % Metaclass for storing gains for the Atlas controller and other parameters which
  % are fixed at compile time for a particular type of plan (walking, manipulating,
  % standing, recovery, etc.) but which can be switched out for a different set when
  % the plan type changes. 
  properties(GetAccess=public, SetAccess=protected)
    body_motion
    whole_body
    vref_integrator
    W_kdot = zeros(3);
    Kp_ang = 0;
    w_slack = 0.05;
    slack_limit = 30;
    w_grf = 0.0;
    Kp_accel = 1.0;
    contact_threshold = 0.001;
    min_knee_angle = 0.7;
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

      obj.whole_body = struct('Kp', 160*ones(r.getNumPositions(), 1),...
                              'damping_ratio', 0.7,...
                              'Kd', zeros(r.getNumPositions(), 1),...
                              'w_qdd', 1e-3 * ones(r.getNumVelocities(), 1),...
                              'integrator', struct('gains', zeros(r.getNumPositions(), 1),...
                                                   'clamps', zeros(r.getNumPositions(), 1),...
                                                   'eta', 0.995),...
                              'qdd_bounds', struct('min', -100*ones(r.getNumVelocities(), 1),...
                                                   'max', 100*ones(r.getNumVelocities(), 1)));
      obj.vref_integrator = struct('zero_ankles_on_contact', false,...
                                      'eta', 0.001);
      obj.whole_body.w_qdd(r.findPositionIndices('back_bkx')) = 0.1;
      obj.whole_body.Kp(r.findPositionIndices('back_bkx')) = 50;

      obj = obj.updateKd();
    end

    function obj = updateKd(obj)
      % Recompute derivative gains based on provided damping ratios
      obj.whole_body.Kd = getDampingGain(obj.whole_body.Kp, 0.7);
      for j = 1:length(obj.body_motion)
        obj.body_motion(j).Kd = getDampingGain(obj.body_motion(j).Kp, 0.7);
      end
    end
  end
end
