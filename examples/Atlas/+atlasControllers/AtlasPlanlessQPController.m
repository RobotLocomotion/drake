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
    all_contacts = struct('groups', {{}}, ... % convenience for indexing into contact groups
                          'point_ind', {{}}, ...
                          'num_points', {{}}, ...
                          'bodies', {{}});
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

      import atlasControllers.PlanlessQPInput2D;
      obj.all_contacts.groups = cell(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body);
      obj.all_contacts.num_points = zeros(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body);
      obj.all_contacts.point_ind = cell(PlanlessQPInput2D.num_support_bodies, PlanlessQPInput2D.contact_groups_per_body);
      obj.all_contacts.bodies = PlanlessQPInput2D.support_body_ids;
      for j = 1:size(obj.all_contacts.groups, 1)
        body_id = PlanlessQPInput2D.support_body_ids(j);
        contact_groups = r.getBody(body_id).collision_geometry_group_names;
        for k = 1:size(obj.all_contacts.groups, 2)
          if k <= length(contact_groups)
            obj.all_contacts.groups{j,k} = contact_groups{k};
            obj.all_contacts.point_ind{j,k} = r.getBody(body_id).collision_geometry_group_indices{k};
            obj.all_contacts.num_points(j,k) = length(obj.all_contacts.point_ind{j,k});
          end
        end
      end

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
        obj.mex_ptr = SharedDataHandle(statelessQPControllermex(0,obj,obj.robot.getMexModelPtr.ptr,getB(obj.robot),r.umin,r.umax,terrain_map_ptr));
      end

    end

    function supp = getPlannedSupports(obj, group_mask)
      support_body_mask = any(group_mask, 2);
      active_rows = find(support_body_mask);
      bodies = obj.all_contacts.bodies(support_body_mask);

      contact_pts = cell(1, length(bodies));
      contact_groups = cell(1, length(bodies));
      num_contact_pts = zeros(1, length(bodies));
      for j = 1:length(contact_pts)
        contact_pts{j} = [obj.all_contacts.point_ind{active_rows(j),group_mask(active_rows(j),:)}];
        contact_groups{j} = obj.all_contacts.groups(active_rows(j),group_mask(active_rows(j),:));
        num_contact_pts(j) = sum(obj.all_contacts.num_points(active_rows(j),:));
      end
      supp = struct('bodies', {bodies},...
                    'contact_pts', {contact_pts},...
                    'contact_groups', {contact_groups},...
                    'num_contact_pts', {num_contact_pts},...
                    'contact_surfaces', {0*bodies});
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
      
      R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;

      q = x(1:obj.numq);
      qd = x(obj.numq+(1:obj.numv));
      mu = qp_input.support_data.mu;
      condof = find(qp_input.whole_body_data.constrained_dof_mask);

      % Run PD on the desired configuration
      qddot_des = obj.qpd.getQddot_des(q, qd,...
                          qp_input.whole_body_data.q_des,...
                          params.whole_body);

      supp = obj.getPlannedSupports(qp_input.support_data.group_mask);
      supp = obj.foot_contact.getActiveSupports(x, supp, contact_sensor,...
        qp_input.support_data.breaking_contact);

      [w_qdd, qddot_des] = obj.kneePD(q, qd, w_qdd, qddot_des, qp_input.support_data.toe_off, params.min_knee_angle);

      all_bodies_vdot = struct('body_id', cell(1,length(qp_input.bodies_data)),...
                               'body_vdot', cell(1,length(qp_input.bodies_data)),...
                               'params', cell(1,length(qp_input.bodies_data)));
      num_tracked_bodies = 0;
      for j = 1:length(qp_input.bodies_data)
        body_id = qp_input.bodies_data(j).body_id;
        if ~isempty(body_id) && params.body_motion(body_id).weight ~= 0
          num_tracked_bodies = num_tracked_bodies + 1;
          body_vdot = obj.body_accel_pd.getBodyVdot(t, x, qp_input.bodies_data(j), params.body_motion(body_id));
          all_bodies_vdot(num_tracked_bodies).body_id = body_id; 
          all_bodies_vdot(num_tracked_bodies).body_vdot = body_vdot;
          all_bodies_vdot(num_tracked_bodies).params = params.body_motion(body_id);
        end
      end
      all_bodies_vdot = all_bodies_vdot(1:num_tracked_bodies);

      qdd_lb =-500*ones(1,obj.numq);
      qdd_ub = 500*ones(1,obj.numq);

      % fprintf(1, ' non_mex: %f\n', toc(t0));

      if (obj.use_mex==0 || obj.use_mex==2)
        kinsol = doKinematics(r,q,false,true,qd);

        if length(supp) < 1
          keyboard();
        end
        active_supports = supp.bodies;
        active_contact_groups = supp.contact_groups;
        num_active_contacts = supp.num_contact_pts;

        dim = 3; % 3D
        nd = 4; % for friction cone approx, hard coded for now
        float_idx = 1:6; % indices for floating base dofs
        act_idx = 7:nq; % indices for actuated dofs

        [H,C,B] = manipulatorDynamics(r,q,qd);

        H_float = H(float_idx,:);
        C_float = C(float_idx);

        H_act = H(act_idx,:);
        C_act = C(act_idx);
        B_act = B(act_idx,:);

        [xcom,Jcom] = getCOM(r,kinsol);
        % lcmgl = LCMGLClient('actual com');
        % lcmgl.glColor3f(0.8, 1, 0);
        % lcmgl.sphere([xcom(1:2); 0], 0.02, 20, 20);
        % lcmgl.switchBuffers();


        include_angular_momentum = any(any(params.W_kdot));

        if include_angular_momentum
          [A,Adot] = getCMM(r,kinsol,qd);
        end

        Jcomdot = forwardJacDot(r,kinsol,0);
        if length(x0)==4
         Jcom = Jcom(1:2,:); % only need COM x-y
         Jcomdot = Jcomdot(1:2,:);
        end

        if ~isempty(active_supports)
          nc = sum(num_active_contacts);
          Dbar = [];
          for j=1:length(active_supports)
            [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet,...
              'body_idx',[1,active_supports(j)],'collision_groups',active_contact_groups(j)));
            Dbar = [Dbar, vertcat(JB{:})']; % because contact constraints seems to ignore the collision_groups option
          end

          Dbar_float = Dbar(float_idx,:);
          Dbar_act = Dbar(act_idx,:);

          terrain_pts = getTerrainContactPoints(r,active_supports,active_contact_groups);
          [~,Jp,Jpdot] = terrainContactPositions(r,kinsol,terrain_pts,true);
          Jp = sparse(Jp);
          Jpdot = sparse(Jpdot);

          if length(x0)==4
            xlimp = [xcom(1:2); Jcom*qd]; % state of LIP model
          else
            xlimp = [xcom;Jcom*qd];
          end
          x_bar = xlimp - x0;
        else
          nc = 0;
        end
        neps = nc*dim;


        %----------------------------------------------------------------------
        % Build handy index matrices ------------------------------------------

        nf = nc*nd; % number of contact force variables
        nparams = nq+nf+neps;
        Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
        Ibeta = zeros(nf,nparams); Ibeta(:,nq+(1:nf)) = eye(nf);
        Ieps = zeros(neps,nparams);
        Ieps(:,nq+nf+(1:neps)) = eye(neps);


        %----------------------------------------------------------------------
        % Set up problem constraints ------------------------------------------

        lb = [qdd_lb zeros(1,nf)   -params.slack_limit*ones(1,neps)]'; % qddot/contact forces/slack vars
        ub = [qdd_ub 1e3*ones(1,nf) params.slack_limit*ones(1,neps)]';

        Aeq_ = cell(1,length(all_bodies_vdot)+3+1);
        beq_ = cell(1,5);
        Ain_ = cell(1,2+length(all_bodies_vdot)*2);
        bin_ = cell(1,2+length(all_bodies_vdot)*2);

        % constrained dynamics
        if nc>0
          Aeq_{1} = H_float*Iqdd - Dbar_float*Ibeta;
        else
          Aeq_{1} = H_float*Iqdd;
        end
        beq_{1} = -C_float;

        % input saturation constraints
        % u=B_act'*(H_act*qdd + C_act - Jz_act'*z - Dbar_act*beta)

        if nc>0
          Ain_{1} = B_act'*(H_act*Iqdd - Dbar_act*Ibeta);
        else
          Ain_{1} = B_act'*H_act*Iqdd;
        end
        bin_{1} = -B_act'*C_act + r.umax;
        Ain_{2} = -Ain_{1};
        bin_{2} = B_act'*C_act - r.umin;

        constraint_index = 3;
        for ii=1:length(all_bodies_vdot)
          body_id = all_bodies_vdot(ii).body_id;
          [~,Jb] = forwardKin(r,kinsol,body_id,[0;0;0],1);
          Jbdot = forwardJacDot(r,kinsol,body_id,[0;0;0],1);
          Ain_{constraint_index} = Jb*Iqdd;
          bin_{constraint_index} = -Jbdot*qd + params.body_motion(body_id).accel_bounds.max;
          constraint_index = constraint_index + 1;
          Ain_{constraint_index} = -Jb*Iqdd;
          bin_{constraint_index} = Jbdot*qd - params.body_motion(body_id).accel_bounds.min;
          constraint_index = constraint_index + 1;
        end

        if nc > 0
          % relative acceleration constraint
          Aeq_{2} = Jp*Iqdd + Ieps;
          beq_{2} = -Jpdot*qd - params.Kp_accel*Jp*qd;
        end

        eq_count=3;

        for ii=1:length(all_bodies_vdot)
          body_id = all_bodies_vdot(ii).body_id;
          if all_bodies_vdot(ii).params.weight < 0
            body_vdot = all_bodies_vdot(ii).body_vdot;
            if ~any(active_supports==body_id)
              [~,J] = forwardKin(r,kinsol,body_id,[0;0;0],1);
              Jdot = forwardJacDot(r,kinsol,body_id,[0;0;0],1);
              cidx = ~isnan(body_vdot);
              Aeq_{eq_count} = J(cidx,:)*Iqdd;
              beq_{eq_count} = -Jdot(cidx,:)*qd + body_vdot(cidx);
              eq_count = eq_count+1;
            end
          end
        end

        if ~isempty(condof)
          % add joint acceleration constraints
          conmap = zeros(length(condof),nq);
          conmap(:,condof) = eye(length(condof));
          Aeq_{eq_count} = conmap*Iqdd;
          beq_{eq_count} = qddot_des(condof);
        end

        % linear equality constraints: Aeq*alpha = beq
        Aeq = sparse(vertcat(Aeq_{:}));
        beq = vertcat(beq_{:});

        % linear inequality constraints: Ain*alpha <= bin
        Ain = sparse(vertcat(Ain_{:}));
        bin = vertcat(bin_{:});
        Ain = Ain(bin~=inf,:);
        bin = bin(bin~=inf);

        if include_angular_momentum
          Ak = A(1:3,:);
          Akdot = Adot(1:3,:);
          k=Ak*qd;
          kdot_des = -obj.Kp_ang*k;
        end

        %----------------------------------------------------------------------
        % QP cost function ----------------------------------------------------
        %
        % min: ybar*Qy*ybar + ubar*R*ubar + (2*S*xbar + s1)*(A*x + B*u) +
        % w_qdd*quad(qddot_ref - qdd) + w_eps*quad(epsilon) +
        % w_grf*quad(beta) + quad(kdot_des - (A*qdd + Adot*qd))
        if nc > 0
          Hqp = Iqdd'*Jcom'*R_DQyD_ls*Jcom*Iqdd;
          Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + diag(w_qdd);
          if include_angular_momentum
            Hqp = Hqp + Iqdd'*Ak'*obj.W_kdot*Ak*Iqdd;
          end

          fqp = xlimp'*C_ls'*Qy*D_ls*Jcom*Iqdd;
          fqp = fqp + qd'*Jcomdot'*R_DQyD_ls*Jcom*Iqdd;
          fqp = fqp + (x_bar'*S + 0.5*s1')*B_ls*Jcom*Iqdd;
          fqp = fqp - u0'*R_ls*Jcom*Iqdd;
          fqp = fqp - y0'*Qy*D_ls*Jcom*Iqdd;
          fqp = fqp - (w_qdd.*qddot_des)'*Iqdd;
          if include_angular_momentum
            fqp = fqp + qd'*Akdot'*obj.W_kdot*Ak*Iqdd;
            fqp = fqp - kdot_des'*obj.W_kdot*Ak*Iqdd;
          end

          Hqp(nq+(1:nf),nq+(1:nf)) = params.w_grf*eye(nf);
          Hqp(nparams-neps+1:end,nparams-neps+1:end) = params.w_slack*eye(neps);
        else
          Hqp = Iqdd'*Iqdd;
          fqp = -qddot_des'*Iqdd;
        end

        for ii=1:length(all_bodies_vdot)
          body_id = all_bodies_vdot(ii).body_id;
          w = all_bodies_vdot(ii).params.weight;
          if w>0
            body_vdot = all_bodies_vdot(ii).body_vdot;
            if ~any(active_supports==body_id)
              [~,J] = forwardKin(r,kinsol,body_id,[0;0;0],1);
              Jdot = forwardJacDot(r,kinsol,body_id,[0;0;0],1);
              cidx = ~isnan(body_vdot);
              Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + w*J(cidx,:)'*J(cidx,:);
              fqp = fqp + w*(qd'*Jdot(cidx,:)'- body_vdot(cidx)')*J(cidx,:)*Iqdd;
            end
          end
        end

        %----------------------------------------------------------------------
        % Solve QP ------------------------------------------------------------

        REG = 1e-8;

        IR = eye(nparams);
        lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
        Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
        bin_fqp = [bin; -lb(lbind); ub(ubind)];

        % call fastQPmex first
        QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq), ...
                    params.w_grf*ones(nf,1) + REG*ones(nf,1), ...
                    params.w_slack*ones(neps,1) + REG*ones(neps,1)};
        Aeq_fqp = full(Aeq);
        % NOTE: model.obj is 2* f for fastQP!!!
        [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

        if info_fqp<0
          % then call gurobi
          % disp('QPController: failed over to gurobi');
          model.Q = sparse(Hqp + REG*eye(nparams));
          model.A = [Aeq; Ain];
          model.rhs = [beq; bin];
          model.sense = [obj.eq_array(1:length(beq)); obj.ineq_array(1:length(bin))];
          model.lb = lb;
          model.ub = ub;

          model.obj = fqp;
          if obj.gurobi_options.method==2
            % see drake/algorithms/QuadraticProgram.m solveWGUROBI
            model.Q = .5*model.Q;
          end

          if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
            keyboard;
          end

    %         qp_tic = tic;
          result = gurobi(model,obj.gurobi_options);
    %         qp_toc = toc(qp_tic);
    %         fprintf('QP solve: %2.4f\n',qp_toc);

          alpha = result.x;
        end

        qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
        obj.controller_data.qp_active_set = qp_active_set;

        %----------------------------------------------------------------------
        % Solve for inputs ----------------------------------------------------

        qdd = alpha(1:nq);
        if nc>0
          beta = alpha(nq+(1:nf));
          u = B_act'*(H_act*qdd + C_act - Dbar_act*beta);
        else
          u = B_act'*(H_act*qdd + C_act);
        end
        y = u;

        if (obj.use_mex==2)
          des.y = y;
        end

        % % compute V,Vdot
        % if (nc>0)
        %   %V = x_bar'*S*x_bar + s1'*x_bar + s2;
        %   %Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*Sdot*x_bar + x_bar'*s1dot + s2dot;
        %   % note for ZMP dynamics, S is constant so Sdot=0
        %   Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*s1dot + s2dot;
        % end
      end


      if (obj.use_mex==1 || obj.use_mex==2)
        if obj.foot_contact.using_flat_terrain
          height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
        else
          height = 0;
        end

        if (obj.use_mex==1)
          [y,qdd,info_fqp,active_supports,alpha] = statelessQPControllermex(obj.mex_ptr.data,params,obj.solver==0,qddot_des,x,...
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
            statelessQPControllermex(obj.mex_ptr.data,params,obj.solver==0,qddot_des,x,...
            all_bodies_vdot,condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,...
            s1dot,s2dot,x0,u0,y0,qdd_lb,qdd_ub,w_qdd,mu,height);

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
