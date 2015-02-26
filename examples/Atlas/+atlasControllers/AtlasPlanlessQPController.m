classdef AtlasPlanlessQPController 
  properties(SetAccess=private, GetAccess=public);
    body_accel_pd;
    debug;
    debug_pub;
    robot;
    numq;
    numv;
    numbod;
    % all_contacts = struct('groups', {{}}, ... % convenience for indexing into contact groups
    %                       'point_ind', {{}}, ...
    %                       'num_points', {{}}, ...
    %                       'bodies', {{}});
    controller_data
    q_integrator_data
    vref_integrator_data
    knee_ind;
    ankle_inds;
    leg_inds;
    actuated_inds;
    use_mex;
    mex_ptr;
    support_detect_mex_ptr;
    use_bullet = false;
    using_flat_terrain;

    param_sets

    joint_limits = struct('min', [], 'max', []);


    gurobi_options = struct();
    solver = 0;
  end

  methods
    function obj = AtlasPlanlessQPController(r, body_accel_pd, param_sets, options)
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
      [obj.joint_limits.min, obj.joint_limits.max] = r.getJointLimits();
      obj.numq = r.getNumPositions();
      obj.numv = r.getNumVelocities();
      obj.numbod = length(r.getManipulator().body);
      obj.ankle_inds = struct('right', r.findPositionIndices('r_leg_ak'),...
                              'left', r.findPositionIndices('l_leg_ak'));
      obj.leg_inds = struct('right', r.findPositionIndices('r_leg'),...
                            'left', r.findPositionIndices('l_leg'));
      obj.actuated_inds = r.getActuatedJoints();
      import atlasControllers.*;
      import atlasFrames.*;
      obj.body_accel_pd = body_accel_pd;

      obj.knee_ind = struct('right', r.findPositionIndices('r_leg_kny'),...
                            'left', r.findPositionIndices('l_leg_kny'));

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

      if (obj.use_mex>0)
        terrain = getTerrain(r);
        if isa(terrain,'DRCTerrainMap')
          terrain_map_ptr = terrain.map_handle.getPointerForMex();
          obj.using_flat_terrain = false;
        else
          terrain_map_ptr = 0;
          obj.using_flat_terrain = true;
        end
        obj.mex_ptr = SharedDataHandle(constructQPDataPointer(obj.robot.getMexModelPtr.ptr,...
          obj.robot.getB(), obj.robot.umin, obj.robot.umax,...
          terrain_map_ptr, obj.gurobi_options));
      end

      if exist('supportDetectmex','file')~=3
        error('can''t find supportDetectmex.  did you build it?');
      end      
      obj.support_detect_mex_ptr = SharedDataHandle(supportDetectmex(0,obj.robot.getMexModelPtr.ptr,terrain_map_ptr));

    end

    function supp = getActiveSupports(obj, support_data, contact_sensor, contact_threshold)
      % Combine information from the plan, the kinematic sensors, and the force sensors (if available)
      % to decide which bodies are in active support right now. 

      % first, figure out which bodies need to be checked for kinematic contact
      % We'll do this by reading out the contact logic maps for each body. For
      % a description of what these do, see QPInput2D.m
      % we only need to check kinematic contact if it would affect our decision about
      % whether or not the body is in contact. 
      needs_kin_check = false(obj.numbod, 1);
      for j = 1:length(support_data)
        body_id = support_data(j).body_id;
        if ((support_data(j).support_logic_map(2) ~= support_data(j).support_logic_map(1)) && (~contact_sensor(body_id))) || ...
           ((support_data(j).support_logic_map(4) ~= support_data(j).support_logic_map(3)) && (contact_sensor(body_id)))
          needs_kin_check(body_id) = true;
        end
      end

      obj.robot.warning_manager.warnOnce('Drake:NotDoingSupportDetect', 'support detect not implemented here');
      kin_sensor = false(obj.numbod, 1);
      kin_sensor(needs_kin_check) = true;

      active_support_mask = false(length(support_data), 1);
      for j = 1:length(support_data)
        body_id = support_data(j).body_id;
        active_support_mask(j) = obj.applyContactLogic(support_data(j).support_logic_map, contact_sensor(body_id), kin_sensor(body_id));
      end
      supp = support_data(active_support_mask);

      for j = 1:length(supp)
        supp(j).num_contact_pts = size(supp(j).contact_pts, 2);
        supp(j).contact_surfaces = 0;
      end
    end

    function qddot_des = wholeBodyPID(obj, t, q, qd, q_des, params)
      if isnan(obj.q_integrator_data.t_prev)
        obj.q_integrator_data.t_prev = t;
      end
      dt = t - obj.q_integrator_data.t_prev;
      obj.q_integrator_data.t_prev = t;
      new_int_state = params.integrator.eta * obj.q_integrator_data.state + params.integrator.gains .* (q_des - q) * dt;
      if any(new_int_state)
        new_int_state = max(-params.integrator.clamp, min(params.integrator.clamp, new_int_state));
        q_des = q_des + new_int_state;
        q_des = max(obj.joint_limits.min - params.integrator.clamp,...
                    min(obj.joint_limits.max + params.integrator.clamp, q_des)); % allow it to go delta above and below jlims
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
      if toe_off.right
        r_kny_qdd_des = kp*(min_knee_angle-q(obj.knee_ind.right)) - kd*qd(obj.knee_ind.right);
        qddot_des(obj.knee_ind.right) = r_kny_qdd_des;
        w_qdd(obj.knee_ind.right) = 1;
      elseif q(obj.knee_ind.right) < min_knee_angle
        w_qdd(obj.knee_ind.right) = 1e-4;
      end
      if toe_off.left
        l_kny_qdd_des = kp*(min_knee_angle-q(obj.knee_ind.left)) - kd*qd(obj.knee_ind.left);
        qddot_des(obj.knee_ind.left) = l_kny_qdd_des;
        w_qdd(obj.knee_ind.left) = 1;
      elseif q(obj.knee_ind.left) < min_knee_angle
        w_qdd(obj.knee_ind.left) = 1e-4;
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
        qd_int(obj.ankle_inds.left) = 0;
      end
      if params.zero_ankles_on_contact && fc.right
        qd_int(obj.ankle_inds.right) = 0;
      end

      fc_prev = obj.vref_integrator_data.fc_prev;
      if fc_prev.right ~= fc.right
        % contact state changed, reset integrated velocities
        qd_int(obj.leg_inds.right) = qd(obj.leg_inds.right);
      end
      if fc_prev.left ~= fc.left
        qd_int(obj.leg_inds.left) = qd(obj.leg_inds.left);
      end

      obj.vref_integrator_data.fc_prev = fc;
      obj.vref_integrator_data.state = qd_int;

      qd_err = qd_int - qd;

      % do not velocity control ankles when in contact
      if params.zero_ankles_on_contact && fc.left
        qd_err(obj.ankle_inds.left) = 0;
      end
      if params.zero_ankles_on_contact && fc.right
        qd_err(obj.ankle_inds.right) = 0;
      end

      delta_max = 1.0;
      v_ref = max(-delta_max,min(delta_max,qd_err(obj.actuated_inds)));
    end

    function [y, v_ref] = tick(obj, t, x, qp_input, foot_contact_sensor)
      t0 = tic();

      % Unpack variable names (just to make our code a bit easier to read)
      A_ls = qp_input.zmp_data.A;
      B_ls = qp_input.zmp_data.B;
      C_ls = qp_input.zmp_data.C;
      D_ls = qp_input.zmp_data.D;
      Qy = qp_input.zmp_data.Qy;
      R_ls = qp_input.zmp_data.R;
      S = qp_input.zmp_data.S;
      s1 = qp_input.zmp_data.s1;
      s1dot = 0;
      s2dot = 0;
      x0 = qp_input.zmp_data.x0;
      y0 = qp_input.zmp_data.y0;
      u0 = qp_input.zmp_data.u0;
      ctrl_data = obj.controller_data;
      r = obj.robot;
      nq = obj.numq;

      params = obj.param_sets.(char(qp_input.param_set_name));
      w_qdd = params.whole_body.w_qdd;

      % lcmgl = LCMGLClient('desired zmp');
      % lcmgl.glColor3f(0, 0.7, 1.0);
      % lcmgl.sphere([y0; 0], 0.02, 20, 20);
      % lcmgl.switchBuffers();
      
      q = x(1:obj.numq);
      qd = x(obj.numq+(1:obj.numv));
      
      mu = qp_input.support_data(1).mu;
      condof = params.whole_body.constrained_dofs;

      % Run PD on the desired configuration
      qddot_des = obj.wholeBodyPID(t, q, qd, qp_input.whole_body_data.q_des, params.whole_body);

      contact_sensor = zeros(obj.numbod, 1);
      if foot_contact_sensor(1) > 0.5
        contact_sensor(obj.foot_body_id.left) = 1;
      end
      if foot_contact_sensor(2) > 0.5
        contact_sensor(obj.foot_body_id.right) = 1;
      end
      supp = obj.getActiveSupports(qp_input.support_data, contact_sensor, params.contact_threshold);
      % Messy reorganization for compatibility with supportDetectMex
      bods = zeros(1, length(supp));
      pts = cell(1, length(supp));
      surfaces = cell(1, length(supp));
      for j = 1:length(supp)
        bods(j) = supp(j).body_id;
        pts{j} = supp(j).contact_pts;
        surfaces{j} = supp(j).contact_surfaces;
      end
      supp = struct('bodies', {bods}, 'contact_pts', {pts}, 'contact_surfaces', {surfaces});

      [w_qdd, qddot_des] = obj.kneePD(q, qd, w_qdd, qddot_des, struct('right', false, 'left', false), params.min_knee_angle);

      all_bodies_vdot = struct('body_id', cell(1,length(qp_input.body_motion_data)),...
                               'body_vdot', cell(1,length(qp_input.body_motion_data)),...
                               'params', cell(1,length(qp_input.body_motion_data)));
      num_tracked_bodies = 0;
      for j = 1:length(qp_input.body_motion_data)
        body_id = qp_input.body_motion_data(j).body_id;
        if ~isempty(body_id) && params.body_motion(body_id).weight ~= 0
          num_tracked_bodies = num_tracked_bodies + 1;
          [body_des, body_v_des, body_vdot_des] = evalCubicSplineSegment(t - qp_input.body_motion_data(j).ts(1), qp_input.body_motion_data(j).coefs);
          body_vdot = obj.body_accel_pd(r, x, body_id,...
                                        body_des, body_v_des, body_vdot_des, params.body_motion(body_id));
          all_bodies_vdot(num_tracked_bodies).body_id = body_id; 
          all_bodies_vdot(num_tracked_bodies).body_vdot = body_vdot;
          all_bodies_vdot(num_tracked_bodies).params = params.body_motion(body_id);
        end
      end
      all_bodies_vdot = all_bodies_vdot(1:num_tracked_bodies);

      qdd_lb =-500*ones(1,obj.numq);
      qdd_ub = 500*ones(1,obj.numq);

      % fprintf(1, ' non_mex: %f\n', toc(t0));

      use_fastqp = obj.solver == 0;

      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end

      if (obj.use_mex==0 || obj.use_mex==2)
        [y, qdd, info_fqp, active_supports,...
          alpha, Hqp, fqp, Aeq, beq, Ain, bin,lb,ub] = atlasControllers.setupAndSolveQP(r, params, use_fastqp,...
                                             qddot_des, x, all_bodies_vdot,...
                                             condof, supp, A_ls, B_ls, Qy,...
                                             R_ls, C_ls, D_ls, S, s1,...
                                             s1dot, s2dot, x0, u0, y0, ...
                                             qdd_lb, qdd_ub, w_qdd, ...
                                             mu, height,obj.use_bullet,...
                                             ctrl_data, obj.gurobi_options);
      end

      if (obj.use_mex==1 || obj.use_mex==2)

        if (obj.use_mex==1)
          [y,qdd,info_fqp,active_supports,alpha] = statelessQPControllermex(obj.mex_ptr.data,params,use_fastqp,qddot_des,x,...
              all_bodies_vdot,condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,...
              S,s1,s1dot,s2dot,x0,u0,y0,qdd_lb,qdd_ub,w_qdd,mu,height);

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
          [y_mex,mex_qdd,info_mex,active_supports_mex,~,Hqp_mex,fqp_mex,...
            Aeq_mex,beq_mex,Ain_mex,bin_mex,Qf,Qeps] = ...
            statelessQPControllermex(obj.mex_ptr.data,params,use_fastqp,qddot_des,x,...
            all_bodies_vdot,condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,...
            s1dot,s2dot,x0,u0,y0,qdd_lb,qdd_ub,w_qdd,mu,height);

          num_active_contacts = zeros(1, length(supp.bodies));
          for j = 1:length(supp.contact_pts)
            num_active_contacts(j) = size(supp.contact_pts{j}, 2);
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
          valuecheck(Hqp,blkdiag(Hqp_mex,diag(Qf),diag(Qeps)),1e-6);
          if (nc>0)
            valuecheck(active_supports_mex,active_supports);
          end
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

      if nargout >= 2
        v_ref = obj.velocityReference(t, q, qd, qdd, foot_contact_sensor, params.vref_integrator);
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
                betaj = beta((j - 1) * nd + (1 : nd * supp.num_contact_pts(j)));
                contact_positionsj = r.getBody(active_supports(j)).getTerrainContactPoints();
                forcej = zeros(3, 1);
                torquej = zeros(3, 1);
                min_contact_position_z = inf;
                for k = 1 : supp.num_contact_pts(j)
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
