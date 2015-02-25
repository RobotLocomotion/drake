classdef AtlasPlanlessQPController 
  properties(SetAccess=private, GetAccess=public);
    qpd;
    foot_contact;
    body_accel_pd;
    debug;
    debug_pub;
    robot;
    numq;
    numv;
    % all_contacts = struct('groups', {{}}, ... % convenience for indexing into contact groups
    %                       'point_ind', {{}}, ...
    %                       'num_points', {{}}, ...
    %                       'bodies', {{}});
    controller_data
    knee_ind;
    use_mex;
    mex_ptr;
    use_bullet = false;

    param_sets


    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    gurobi_options = struct();
    solver = 0;
  end

  methods
    function obj = AtlasPlanlessQPController(r, qpd, foot_contact, body_accel_pd, param_sets, options)
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
      obj.numq = r.getNumPositions();
      obj.numv = r.getNumVelocities();
      import atlasController.*;
      import atlasFrames.*;
      obj.qpd = qpd;
      obj.foot_contact = foot_contact;
      obj.body_accel_pd = body_accel_pd;

      obj.knee_ind = struct('right', r.findPositionIndices('r_leg_kny'),...
                            'left', r.findPositionIndices('l_leg_kny'));

      if obj.debug
        obj.debug_pub = ControllerDebugPublisher('CONTROLLER_DEBUG');
      end

      obj.controller_data = PlanlessQPControllerData(struct('infocount', 0,...
                                                     'qp_active_set', []));

      % import atlasControllers.PlanlessQPInput2D;
      % obj.all_contacts.groups = cell(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body);
      % obj.all_contacts.num_points = zeros(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body);
      % obj.all_contacts.point_ind = cell(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body);
      % obj.all_contacts.bodies = PlanlessQPInput2D.support_body_ids;
      % for j = 1:size(obj.all_contacts.groups, 1)
      %   body_id = PlanlessQPInput2D.support_body_ids(j);
      %   contact_groups = r.getBody(body_id).collision_geometry_group_names;
      %   for k = 1:size(obj.all_contacts.groups, 2)
      %     if k <= length(contact_groups)
      %       obj.all_contacts.groups{j,k} = contact_groups{k};
      %       obj.all_contacts.point_ind{j,k} = r.getBody(body_id).collision_geometry_group_indices{k};
      %       obj.all_contacts.num_points(j,k) = length(obj.all_contacts.point_ind{j,k});
      %     end
      %   end
      % end

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
        else
          terrain_map_ptr = 0;
        end
        obj.mex_ptr = SharedDataHandle(constructQPDataPointer(obj.robot.getMexModelPtr.ptr,...
          obj.robot.getB(), obj.robot.umin, obj.robot.umax,...
          terrain_map_ptr, obj.gurobi_options));
      end

    end

    function supp = getPlannedSupports(obj, active_supports)
      supp = active_supports;
      for j = 1:length(supp)
        supp(j).num_contact_pts = size(supp(j).contact_pts, 2);
        supp(j).contact_surfaces = 0;
      end
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

    function [y, qdd] = tick(obj, t, x, qp_input, contact_sensor)
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

      params = obj.param_sets.(qp_input.param_set_name);
      w_qdd = params.whole_body.w_qdd;

      % lcmgl = LCMGLClient('desired zmp');
      % lcmgl.glColor3f(0, 0.7, 1.0);
      % lcmgl.sphere([y0; 0], 0.02, 20, 20);
      % lcmgl.switchBuffers();
      
      q = x(1:obj.numq);
      qd = x(obj.numq+(1:obj.numv));
      mu = qp_input.support_data.mu;
      condof = params.whole_body.constrained_dofs;

      % Run PD on the desired configuration
      qddot_des = obj.qpd.getQddot_des(q, qd,...
                          qp_input.whole_body_data.q_des,...
                          params.whole_body);

      supp = obj.getPlannedSupports(qp_input.support_data.active_supports);
      supp = obj.foot_contact.getActiveSupports(x, supp, contact_sensor,...
        qp_input.support_data.breaking_contact);
      % Messy reorganization for compatibility with supportDetectMex
      supp = struct('bodies', {[supp.body_id]}, 'contact_pts', {{supp.contact_pts}}, 'contact_surfaces', {{supp.contact_surfaces}});

      [w_qdd, qddot_des] = obj.kneePD(q, qd, w_qdd, qddot_des, qp_input.support_data.toe_off, params.min_knee_angle);

      all_bodies_vdot = struct('body_id', cell(1,length(qp_input.bodies_data)),...
                               'body_vdot', cell(1,length(qp_input.bodies_data)),...
                               'params', cell(1,length(qp_input.bodies_data)));
      num_tracked_bodies = 0;
      for j = 1:length(qp_input.bodies_data)
        body_id = qp_input.bodies_data(j).body_id;
        if ~isempty(body_id) && params.body_motion(body_id).weight ~= 0
          num_tracked_bodies = num_tracked_bodies + 1;
          [body_des, body_v_des, body_vdot_des] = evalCubicSplineSegment(t - qp_input.bodies_data(j).ts(1), qp_input.bodies_data(j).coefs);
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

      if obj.foot_contact.using_flat_terrain
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
end
