classdef AtlasQPController < QPController
  % A QP-based balancing and walking controller that exploits TV-LQR solutions
  % for (time-varing) linear COM/ZMP dynamics. Includes logic specific to
  % atlas/bipeds for raising the heel while walking.
  methods
  function obj = AtlasQPController(r,body_accel_input_frames,controller_data,options)
    % @param r rigid body manipulator instance
    % @param body_accel_input_frames cell array or coordinate frames for
    %    desired body accelerations. coordinates are ordered as:
    %    [body_index, xdot, ydot, zdot, roll_dot, pitch_dot, yaw_dot]
    % @param controller_data QPControllerData object containing the matrices that
    % define the underlying linear system, the ZMP trajectory, Riccati
    % solution, etc.
    % @param options structure for specifying objective weights, slack
    % bounds, etc.

    obj = obj@QPController(r,body_accel_input_frames,controller_data,options);
    
    if isfield(options,'debug')
      typecheck(options.debug,'logical');
      sizecheck(options.debug,1);
      obj.debug = options.debug;
    else
      obj.debug = false;
    end

    if obj.debug
      obj.debug_pub = ControllerDebugPublisher('CONTROLLER_DEBUG');
    end
    
    obj.r_knee_idx = r.findPositionIndices('r_leg_kny');
    obj.l_knee_idx = r.findPositionIndices('l_leg_kny');
    if isfield(options,'min_knee_angle')
      sizecheck(options.min_knee_angle,1);
      typecheck(options.min_knee_angle,'double');
      obj.min_knee_angle = options.min_knee_angle;
    else
      obj.min_knee_angle = 0.0;
    end

    obj.controller_data.left_toe_off = false;
    obj.controller_data.right_toe_off = false;
  end

  function varargout=mimoOutput(obj,t,~,varargin)
    %out_tic = tic;

    ctrl_data = obj.controller_data;

    x = varargin{1};
    qddot_des = varargin{2};

    r = obj.robot;
    nq = obj.numq;
    q = x(1:nq);
    qd = x(nq+(1:nq));

    %----------------------------------------------------------------------
    % Linear system/LQR terms ---------------------------------------------
    if ctrl_data.A_is_time_varying
      A_ls = fasteval(ctrl_data.A,t);
    else
      A_ls = ctrl_data.A;
    end
    if ctrl_data.B_is_time_varying
      B_ls = fasteval(ctrl_data.B,t);
    else
      B_ls = ctrl_data.B;
    end
    if ctrl_data.C_is_time_varying
      C_ls = fasteval(ctrl_data.C,t);
    else
      C_ls = ctrl_data.C;
    end
    if ctrl_data.D_is_time_varying
      D_ls = fasteval(ctrl_data.D,t);
    else
      D_ls = ctrl_data.D;
    end
    Qy = ctrl_data.Qy;
    R_ls = ctrl_data.R;
    if (ctrl_data.lqr_is_time_varying)
      if isa(ctrl_data.S,'Trajectory')
        S = fasteval(ctrl_data.S,t);
%       Sdot = fasteval(ctrl_data.Sdot,t);
      else
        S = ctrl_data.S;
%       Sdot = 0*S;
      end
      s1 = fasteval(ctrl_data.s1,t);
%       s2 = fasteval(ctrl_data.s2,t);
%       s1dot = fasteval(ctrl_data.s1dot,t);
%       s2dot = fasteval(ctrl_data.s2dot,t);
      s1dot = 0*s1;
      s2dot = 0;
      x0 = fasteval(ctrl_data.x0,t);
      y0 = fasteval(ctrl_data.y0,t);
      u0 = fasteval(ctrl_data.u0,t);
    else
      S = ctrl_data.S;
      s1 = ctrl_data.s1;
%       s2 = ctrl_data.s2;
%       Sdot = 0*S;
      s1dot = 0*s1;
      s2dot = 0;
      x0 = ctrl_data.x0;
      y0 = ctrl_data.y0;
      u0 = ctrl_data.u0;
    end
    % plan shifting for ZMP sys --- TODO: generalize
    x0 = x0 - [ctrl_data.plan_shift(1:2);0;0];
    y0 = y0 - ctrl_data.plan_shift(1:2);

    % lcmgl = LCMGLClient('desired zmp');
    % lcmgl.glColor3f(0, 0.7, 1.0);
    % lcmgl.sphere([y0; 0], 0.02, 20, 20);
    % lcmgl.switchBuffers();
    
    mu = ctrl_data.mu;
    R_DQyD_ls = R_ls + D_ls'*Qy*D_ls;

    condof = ctrl_data.constrained_dofs; % dof indices for which qdd_des is a constraint

    fc = varargin{3};

    % TODO: generalize this again to arbitrary body contacts
    support_bodies = [];
    contact_pts = {};
    contact_groups = {};
    n_contact_pts = [];
    ind = 1;

    supp_idx = find(ctrl_data.support_times<=t,1,'last');
    plan_supp = ctrl_data.supports(supp_idx);

    lfoot_plan_supp_ind = plan_supp.bodies==r.foot_body_id.left;
    rfoot_plan_supp_ind = plan_supp.bodies==r.foot_body_id.right;
    if fc(1)>0
      support_bodies(ind) = r.foot_body_id.left;
      if q(obj.l_knee_idx) < obj.min_knee_angle
        j = find([ctrl_data.link_constraints(2).ts] > t, 1, 'first');
        if ctrl_data.link_constraints(2).toe_off_allowed(j)
          plan_supp = r.left_toe_right_full_support;
          obj.controller_data.left_toe_off = true;
        end
      end
      contact_groups{ind} = plan_supp.contact_groups{lfoot_plan_supp_ind};
      contact_pts{ind} = plan_supp.contact_pts{lfoot_plan_supp_ind};
      n_contact_pts(ind) = plan_supp.num_contact_pts(lfoot_plan_supp_ind);
      ind=ind+1;
    else
      obj.controller_data.left_toe_off = false;
    end
    if fc(2)>0
      support_bodies(ind) = r.foot_body_id.right;
      if q(obj.r_knee_idx) < obj.min_knee_angle
        j = find([ctrl_data.link_constraints(1).ts] > t, 1, 'first');
        if ctrl_data.link_constraints(1).toe_off_allowed(j)
          plan_supp = r.left_full_right_toe_support;
          obj.controller_data.right_toe_off = true;
        end
      end
      contact_groups{ind} = plan_supp.contact_groups{rfoot_plan_supp_ind};
      contact_pts{ind} = plan_supp.contact_pts{rfoot_plan_supp_ind};
      n_contact_pts(ind) = plan_supp.num_contact_pts(rfoot_plan_supp_ind);
    else
      obj.controller_data.right_toe_off = false;
    end
    obj.controller_data.supports(supp_idx) = plan_supp;

    supp.bodies = support_bodies;
    supp.contact_pts = contact_pts;
    supp.contact_groups = contact_groups;
    supp.num_contact_pts = n_contact_pts;
    
    qdd_lb =-500*ones(1,nq);
    qdd_ub = 500*ones(1,nq);
    w_qdd = obj.w_qdd;
    kp = 40;
    kd = 4;
    if obj.controller_data.right_toe_off
      r_kny_qdd_des = kp*(obj.min_knee_angle-q(obj.r_knee_idx)) - kd*qd(obj.r_knee_idx);
      qddot_des(obj.r_knee_idx) = r_kny_qdd_des;
      w_qdd(obj.r_knee_idx) = 1;
    elseif q(obj.r_knee_idx) < obj.min_knee_angle
      w_qdd(obj.r_knee_idx) = 1e-4;
    end
    if obj.controller_data.left_toe_off
      l_kny_qdd_des = kp*(obj.min_knee_angle-q(obj.l_knee_idx)) - kd*qd(obj.l_knee_idx);
      qddot_des(obj.l_knee_idx) = l_kny_qdd_des;
      w_qdd(obj.l_knee_idx) = 1;
    elseif q(obj.l_knee_idx) < obj.min_knee_angle
      w_qdd(obj.l_knee_idx) = 1e-4;
    end

    if (obj.use_mex==0 || obj.use_mex==2)
      kinsol = doKinematics(r,q,false,true,qd);

      active_supports = supp.bodies;
      active_contact_pts = supp.contact_pts;
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


      include_angular_momentum = any(any(obj.W_kdot));

      if include_angular_momentum
        A = centroidalMomentumMatrix(r, kinsol);
        Adot_times_v = centroidalMomentumMatrixDotTimesVmex(r, kinsol);
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

      lb = [qdd_lb zeros(1,nf)   -obj.slack_limit*ones(1,neps)]'; % qddot/contact forces/slack vars
      ub = [qdd_ub 1e3*ones(1,nf) obj.slack_limit*ones(1,neps)]';

      Aeq_ = cell(1,length(varargin)+1);
      beq_ = cell(1,5);
      Ain_ = cell(1,2+length(obj.body_accel_bounds)*2);
      bin_ = cell(1,2+length(obj.body_accel_bounds)*2);

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
      for ii=1:obj.n_body_accel_bounds
        body_idx = obj.body_accel_bounds(ii).body_idx;
        [~,Jb] = forwardKin(r,kinsol,body_idx,[0;0;0],1);
        Jbdot = forwardJacDot(r,kinsol,body_idx,[0;0;0],1);
        Ain_{constraint_index} = Jb*Iqdd;
        bin_{constraint_index} = -Jbdot*qd + obj.body_accel_bounds(ii).max_acceleration;
        constraint_index = constraint_index + 1;
        Ain_{constraint_index} = -Jb*Iqdd;
        bin_{constraint_index} = Jbdot*qd - obj.body_accel_bounds(ii).min_acceleration;
        constraint_index = constraint_index + 1;
      end

      if nc > 0
        % relative acceleration constraint
        Aeq_{2} = Jp*Iqdd + Ieps;
        beq_{2} = -Jpdot*qd - obj.Kp_accel*Jp*qd;
      end

      eq_count=3;

      for ii=1:obj.n_body_accel_inputs
        if obj.body_accel_input_weights(ii) < 0
          body_input = varargin{ii+3};
          body_ind = body_input(1);
          body_vdot = body_input(2:7);
          if ~any(active_supports==body_ind)
            [~,J] = forwardKin(r,kinsol,body_ind,[0;0;0],1);
            Jdot = forwardJacDot(r,kinsol,body_ind,[0;0;0],1);
            cidx = ~isnan(body_vdot);
            Aeq_{eq_count} = J(cidx,:)*Iqdd;
            beq_{eq_count} = -Jdot(cidx,:)*qd + body_vdot(cidx);
            eq_count = eq_count+1;
          end
        end
      end

      if ~isempty(ctrl_data.constrained_dofs)
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
        Akdot_times_v = Adot_times_v(1:3,:);
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
          fqp = fqp + Akdot_times_v'*obj.W_kdot*Ak*Iqdd;
          fqp = fqp - kdot_des'*obj.W_kdot*Ak*Iqdd;
        end

        Hqp(nq+(1:nf),nq+(1:nf)) = obj.w_grf*eye(nf);
        Hqp(nparams-neps+1:end,nparams-neps+1:end) = obj.w_slack*eye(neps);
      else
        Hqp = Iqdd'*Iqdd;
        fqp = -qddot_des'*Iqdd;
      end

      for ii=1:obj.n_body_accel_inputs
        w = obj.body_accel_input_weights(ii);
        if w>0
          body_input = varargin{ii+3};
          body_ind = body_input(1);
          body_vdot = body_input(2:7);
          if ~any(active_supports==body_ind)
            [~,J] = forwardKin(r,kinsol,body_ind,[0;0;0],1);
            Jdot = forwardJacDot(r,kinsol,body_ind,[0;0;0],1);
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
                  obj.w_grf*ones(nf,1) + REG*ones(nf,1), ...
                  obj.w_slack*ones(neps,1) + REG*ones(neps,1)};
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
      if obj.using_flat_terrain
        height = getTerrainHeight(r,[0;0]); % get height from DRCFlatTerrainMap
      else
        height = 0;
      end

      if (obj.use_mex==1)
        [y,qdd,info_fqp,active_supports,alpha] = QPControllermex(obj.mex_ptr.data,obj.solver==0,qddot_des,x,...
            varargin{4:end},condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,...
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
          QPControllermex(obj.mex_ptr.data,obj.solver==0,qddot_des,x,...
          varargin{4:end},condof,supp,A_ls,B_ls,Qy,R_ls,C_ls,D_ls,S,s1,...
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
      if obj.n_body_accel_inputs > 0
        acc_mat = [varargin{3+(1:obj.n_body_accel_inputs)}];
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

    if obj.output_qdd
      varargout = {y,qdd};
    else
      varargout = {y};
    end
    % % for profiling the entire atlas controller system (see QTrajEvalBlock.m)
    % global qtraj_eval_start_time;
    % toc(qtraj_eval_start_time)
  end
  end

  properties (SetAccess=private)
    debug;
    debug_pub;
    min_knee_angle;
    r_knee_idx;
    l_knee_idx;
  end
end
