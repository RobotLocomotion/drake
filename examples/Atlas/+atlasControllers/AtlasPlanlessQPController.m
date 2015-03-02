classdef AtlasPlanlessQPController 
  properties(SetAccess=private, GetAccess=public);
    debug;
    debug_pub;
    robot;
    controller_data
    q_integrator_data
    vref_integrator_data
    robot_property_cache
    use_mex;
    mex_ptr;
    support_detect_mex_ptr;
    use_bullet = false;
    default_terrain_height = 0;
    param_sets
    gurobi_options = struct();
    solver = 0;
  end

  methods
    function obj = AtlasPlanlessQPController(r, param_sets, options)
      options = applyDefaults(options,...
        struct('debug', false,...
               'use_mex', 1, ...
               'solver', 0),...
        struct('debug', @(x) typecheck(x, 'logical') && sizecheck(x, 1),...
               'use_mex', @isscalar, ...
               'solver', @(x) x == 0 || x == 1));
      for f = fieldnames(options)'
        obj.(f{1}) = options.(f{1});
      end

      obj.robot = r;
      obj.param_sets = param_sets;
      obj.robot_property_cache = atlasUtil.propertyCache(r);
      import atlasControllers.*;
      import atlasFrames.*;

      if obj.debug
        obj.debug_pub = ControllerDebugPublisher('CONTROLLER_DEBUG');
      end

      obj.controller_data = PlanlessQPControllerData(struct('infocount', 0,...
                                                     'qp_active_set', []));
      obj.q_integrator_data = IntegratorData(r);
      obj.vref_integrator_data = VRefIntegratorData(r);

      obj.gurobi_options.outputflag = 0; % not verbose
      if obj.solver==0
        obj.gurobi_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      else
        obj.gurobi_options.method = 0; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
      end
      obj.gurobi_options.presolve = 0;
      % obj.gurobi_options.prepasses = 1;

      if obj.gurobi_options.method == 2
        obj.gurobi_options.bariterlimit = 20; % iteration limit
        obj.gurobi_options.barhomogeneous = 0; % 0 off, 1 on
        obj.gurobi_options.barconvtol = 5e-4;
      end

      terrain = getTerrain(r);
      obj.default_terrain_height = r.getTerrainHeight([0;0]);
      if isa(terrain,'DRCTerrainMap')
        terrain_map_ptr = terrain.map_handle.getPointerForMex();
      else
        terrain_map_ptr = 0;
      end

      obj.mex_ptr = SharedDataHandle(constructQPDataPointerMex(obj.robot.getMexModelPtr.ptr,...
                                                               obj.param_sets,...
                                                               obj.robot_property_cache,...
                                                               obj.robot.getB(),...
                                                               obj.robot.umin,...
                                                               obj.robot.umax,...
                                                               terrain_map_ptr,...
                                                               obj.default_terrain_height,...
                                                               obj.gurobi_options));
    end

    function qddot_des = wholeBodyPID(obj, t, q, qd, q_des, params)
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


    function [w_qdd, qddot_des] = kneePD(obj, q, qd, w_qdd, qddot_des, toe_off, min_knee_angle)
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

    function v_ref = velocityReference(obj, t, q, qd, qdd, foot_contact_sensor, params)
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

    function [y, v_ref] = tick(obj, t, x, qp_input, foot_contact_sensor)

      % obj.robot.warning_manager.warnOnce('Drake:FakeStateDrift', 'Faking state drift');
      % x(1) = x(1) + 0.1*t;

      t0 = tic();

      % Unpack variable names (just to make our code a bit easier to read)
      ctrl_data = obj.controller_data;
      r = obj.robot;
      nq = obj.robot_property_cache.nq;
      nv = obj.robot_property_cache.nv;

      param_set_name = char(qp_input.param_set_name);
      params = obj.param_sets.(param_set_name);

      % Copy the body motion data out of the LCM type (so we can pass it to mex)
      body_motion_data = struct('body_id', cell(1, length(qp_input.body_motion_data)),...
                                'ts', cell(1, length(qp_input.body_motion_data)),...
                                'coefs', cell(1, length(qp_input.body_motion_data)));
      for j = 1:length(qp_input.body_motion_data)
        for f = fieldnames(body_motion_data(j))'
          body_motion_data(j).(f{1}) = qp_input.body_motion_data(j).(f{1});
        end
      end

      % lcmgl = LCMGLClient('desired zmp');
      % lcmgl.glColor3f(0, 0.7, 1.0);
      % lcmgl.sphere([y0; 0], 0.02, 20, 20);
      % lcmgl.switchBuffers();
      
      q = x(1:nq);
      qd = x(nq+(1:nv));
      
      mu = qp_input.support_data(1).mu;

      contact_sensor = zeros(obj.robot_property_cache.num_bodies, 1);
      if foot_contact_sensor(1) > 0.5
        contact_sensor(obj.foot_body_id.left) = 1;
      end
      if foot_contact_sensor(2) > 0.5
        contact_sensor(obj.foot_body_id.right) = 1;
      end
      if isjava(qp_input.support_data)
        % Reconstruct the support data as a matlab struct for mex
        % compatibility
        available_supports = struct('body_id', cell(1, length(qp_input.support_data)),...
                                    'contact_pts', cell(1, length(qp_input.support_data)),...
                                    'support_logic_map', cell(1, length(qp_input.support_data)),...
                                    'mu', cell(1, length(qp_input.support_data)),...
                                    'contact_surfaces', cell(1, length(qp_input.support_data)));
        for j = 1:length(qp_input.support_data)
          available_supports(j).body_id = qp_input.support_data(j).body_id;
          available_supports(j).contact_pts = qp_input.support_data(j).contact_pts;
          available_supports(j).support_logic_map = qp_input.support_data(j).support_logic_map;
          available_supports(j).mu = qp_input.support_data(j).mu;
          available_supports(j).contact_surfaces = qp_input.support_data(j).contact_surfaces;
        end
      else
        available_supports = qp_input.support_data;
      end
      for j = 1:length(available_supports)
        % it's a pain to handle mxLogicals on the c++ side
        available_supports(j).support_logic_map = double(available_supports(j).support_logic_map);
      end

      % [w_qdd, qddot_des] = obj.kneePD(q, qd, w_qdd, qddot_des, struct('right', false, 'left', false), params.min_knee_angle);


      % qdd_lb =-500*ones(1,nq);
      % qdd_ub = 500*ones(1,nq);

      % fprintf(1, ' non_mex: %f\n', toc(t0));

      use_fastqp = obj.solver == 0;

      if (obj.use_mex==0 || obj.use_mex==2)
        % if t >= 0.1
        %   keyboard();
        % end
        all_bodies_vdot = struct('body_id', cell(1,length(body_motion_data)),...
                                 'body_vdot', cell(1,length(body_motion_data)),...
                                 'params', cell(1,length(body_motion_data)));
        num_tracked_bodies = 0;
        fun = fcompare(@atlasControllers.statelessBodyMotionControl,@statelessBodyMotionControlmex);
        for j = 1:length(body_motion_data)
          body_id = body_motion_data(j).body_id;
          if ~isempty(body_id)
            num_tracked_bodies = num_tracked_bodies + 1;
            [body_des, body_v_des, body_vdot_des] = evalCubicSplineSegment(t - body_motion_data(j).ts(1), body_motion_data(j).coefs);
            body_vdot = fun(r, x, body_id,...
                                          body_des, body_v_des, body_vdot_des, params.body_motion(body_id));
            all_bodies_vdot(num_tracked_bodies).body_id = body_id; 
            all_bodies_vdot(num_tracked_bodies).body_vdot = body_vdot;
            all_bodies_vdot(num_tracked_bodies).params = params.body_motion(body_id);
          end
        end
        all_bodies_vdot = all_bodies_vdot(1:num_tracked_bodies);
        mask = getActiveSupportsmex(obj.mex_ptr.data, x, available_supports, contact_sensor, params.contact_threshold, obj.default_terrain_height);
        supp = available_supports(logical(mask));

        % Run PD on the desired configuration
        qddot_des = obj.wholeBodyPID(t, q, qd, qp_input.whole_body_data.q_des, params.whole_body);

        condof = qp_input.whole_body_data.constrained_dofs;

        [y, qdd, info_fqp, active_supports,...
          alpha, Hqp, fqp, Aeq, beq, Ain, bin,lb,ub] = atlasControllers.setupAndSolveQP(r, params, use_fastqp,...
                                             qddot_des, x, all_bodies_vdot,...
                                             condof, supp, qp_input.zmp_data, ...
                                             mu, obj.default_terrain_height,obj.use_bullet,...
                                             ctrl_data, obj.gurobi_options);
        if nargout >= 2
          v_ref = obj.velocityReference(t, q, qd, qdd, foot_contact_sensor, params.vref_integrator);
        end
      end

      if (obj.use_mex==1 || obj.use_mex==2)

        if (obj.use_mex==1)
          t0 = tic();
          [y,qdd,v_ref,info_fqp,active_supports,alpha] = ...
                      statelessQPControllermex(obj.mex_ptr.data,...
                      t,...
                      x,...
                      struct(qp_input.zmp_data),...
                      struct(qp_input.whole_body_data),...
                      body_motion_data,...
                      available_supports,...
                      contact_sensor,...
                      use_fastqp,...
                      mu,...
                      param_set_name);
          fprintf(1, 'mex: %f, ', toc(t0));

          if info_fqp < 0
            ctrl_data.infocount = ctrl_data.infocount+1;
          else
            ctrl_data.infocount = 0;
          end
          if ctrl_data.infocount > 10 && exist('AtlasBehaviorModePublisher','class')
            % kill atlas
            disp('freezing atlas!');
            behavior_pub = AtlasBehaviorModePublisher('ATLAS_BEHAVIOR_COMMAND');
            d.utime = 0;
            d.command = 'freeze';
            behavior_pub.publish(d);
          end

        else
          t0 = tic();
          [y_mex,mex_qdd,vref_mex,info_mex,active_supports_mex,~,Hqp_mex,fqp_mex,...
            Aeq_mex,beq_mex,Ain_mex,bin_mex,Qf,Qeps] = ...
               statelessQPControllermex(obj.mex_ptr.data,...
                                        t,...
                                        x,...
                                        struct(qp_input.zmp_data),...
                                        struct(qp_input.whole_body_data),...
                                        body_motion_data,...
                                        available_supports,...
                                        contact_sensor,...
                                        use_fastqp,...
                                        mu,...
                                        param_set_name);
          fprintf(1, 'mex: %f, ', toc(t0));

          num_active_contacts = zeros(1, length(supp));
          for j = 1:length(supp)
            num_active_contacts(j) = size(supp(j).contact_pts, 2);
          end
          nc = sum(num_active_contacts);
          if (nc>0)
            valuecheck(active_supports_mex,active_supports);
          end

          if size(Hqp_mex,2)==1
            Hqp_mex=diag(Hqp_mex);
          end
          if info_mex < 0
            Hqp_mex=Hqp_mex*2;
            Qf=Qf*2;
            Qeps=Qeps*2;
          end
          if ~valuecheck(Hqp,blkdiag(Hqp_mex,diag(Qf),diag(Qeps)),1e-6);
            keyboard();
          end
          valuecheck(Hqp,blkdiag(Hqp_mex,diag(Qf),diag(Qeps)),1e-6);
          valuecheck(fqp',fqp_mex,1e-6);
          if ~obj.use_bullet
            % contact jacobian rows can be permuted between matlab/mex when
            % using bullet
            valuecheck(Aeq,Aeq_mex(1:length(beq),:),1e-6);
            valuecheck(beq,beq_mex(1:length(beq)),1e-6);
            valuecheck(Ain,Ain_mex(1:length(bin),:),1e-6);
            valuecheck(bin,bin_mex(1:length(bin)),1e-6);
          end
          valuecheck([-lb;ub],bin_mex(length(bin)+1:end),1e-6);
          if info_mex >= 0 && info_fqp >= 0 && ~obj.use_bullet
            % matlab/mex are using different gurobi fallback options, so
            % solutions can be slightly different
            %valuecheck(y,y_mex,1e-3);
            %valuecheck(qdd,mex_qdd,1e-3);
          end
        end
      end

      if obj.debug
        % publish debug
        debug_data.utime = t*1e6;
        debug_data.alpha = alpha;
        debug_data.u = y;
        debug_data.active_supports = active_supports;
        debug_data.info = info_fqp;
        debug_data.qddot_des = qddot_des;
        if obj.use_mex==0 % TODO: update this
          debug_data.active_constraints = qp_active_set;
        else
          debug_data.active_constraints = [];
        end
        debug_data.r_foot_contact = any(r.foot_body_id.right==active_supports);
        debug_data.l_foot_contact = any(r.foot_body_id.left==active_supports);
        if ~isempty(all_bodies_vdot)
          acc_mat = [[all_bodies_vdot.body_id]; all_bodies_vdot.body_vdot];
          debug_data.body_acc_des = reshape(acc_mat,numel(acc_mat),1);
        else
          debug_data.body_acc_des = [];
        end
        debug_data.zmp_err = [0;0];

        debug_data.individual_cops = zeros(3 * length(active_supports), 1);
        if obj.use_mex==0 % TODO: update this
          beta = alpha(nq + (1 : nc * nd));
          if ~isempty(active_supports)
            for j=1:length(active_supports)
              [~,Bj,~,~,normalj] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet,'body_idx',[1,active_supports(j)]));
              normals_identical = ~any(any(bsxfun(@minus, normalj, normalj(:,1))));
              if normals_identical % otherwise computing a COP doesn't make sense
                normalj = normalj(:,1);
                betaj = beta((j - 1) * nd + (1 : nd * supp(j).num_contact_pts));
                contact_positionsj = r.getBody(active_supports(j)).getTerrainContactPoints();
                forcej = zeros(3, 1);
                torquej = zeros(3, 1);
                min_contact_position_z = inf;
                for k = 1 : supp(j).num_contact_pts
                  Bjk = Bj{k};
                  betajk = betaj((k - 1) * nd + (1:nd));
                  contact_positionjk = contact_positionsj(:, k);
                  forcejk = Bjk * betajk;
                  forcej = forcej + forcejk;
                  torquejk = cross(contact_positionjk, forcejk);
                  torquej = torquej + torquejk;
                  if normalj' * contact_positionjk < min_contact_position_z
                    min_contact_position_z = normalj' * contact_positionjk;
                  end
                end
                fzj = normalj' * forcej; % in body frame
                if abs(fzj) > 1e-7
                  normal_torquej = normalj' * torquej; % in body frame
                  tangential_torquej = torquej - normalj * normal_torquej; % in body frame
                  cop_bodyj = cross(normalj, tangential_torquej) / fzj; % in body frame
                  cop_bodyj = cop_bodyj + min_contact_position_z * normalj;
                  cop_worldj = r.forwardKin(kinsol, active_supports(j), cop_bodyj,0);
                else
                  cop_worldj = nan(3, 1);
                end
                debug_data.individual_cops((j - 1) * 3 + (1 : 3)) = cop_worldj;
              end
            end
          end
        end

        obj.debug_pub.publish(debug_data);
      end

      if (0)     % simple timekeeping for performance optimization
        % note: also need to uncomment tic at very top of this method
        out_toc=toc(out_tic);
        persistent average_tictoc average_tictoc_n;
        if isempty(average_tictoc)
          average_tictoc = out_toc;
          average_tictoc_n = 1;
        else
          average_tictoc = (average_tictoc_n*average_tictoc + out_toc)/(average_tictoc_n+1);
          average_tictoc_n = average_tictoc_n+1;
        end
        if mod(average_tictoc_n,50)==0
          fprintf('Average control output duration: %2.4f\n',average_tictoc);
        end
      end
    end
  end
  
  methods(Static)
    function tf = applyContactLogic(support_logic_map, body_force_sensor, body_kin_sensor)
      tf = support_logic_map(2*logical(body_force_sensor) + logical(body_kin_sensor)+1);
    end
  end
  

end
