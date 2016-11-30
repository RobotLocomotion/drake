classdef QPController < MIMODrakeSystem
  % A QP-based balancing and walking controller that exploits TV-LQR solutions
  % for (time-varing) linear COM/ZMP dynamics.
  % optionally supports including angular momentum and body acceleration
  % costs/constraints.
  methods
  function obj = QPController(r,body_accel_input_frames,controller_data,options)
    % @param r rigid body manipulator instance
    % @param body_accel_input_frames cell array or coordinate frames for
    %    desired body accelerations. coordinates are ordered as:
    %    [body_index, xdot, ydot, zdot, roll_dot, pitch_dot, yaw_dot]
    % @param controller_data QPControllerData object containing the matrices that
    % define the underlying linear system, the ZMP trajectory, Riccati
    % solution, etc.
    % @param options structure for specifying objective weights, slack
    % bounds, etc.
    typecheck(r,'Biped');
    typecheck(controller_data,'QPControllerData');

    if nargin>3
      typecheck(options,'struct');
    else
      options = struct();
    end

    qddframe = controller_data.acceleration_input_frame; % input frame for desired qddot

    input_frame = MultiCoordinateFrame({r.getStateFrame,qddframe,atlasFrames.FootContactState,body_accel_input_frames{:}});

    % whether to output generalized accelerations AND inputs (u)
    if ~isfield(options,'output_qdd')
      options.output_qdd = false;
    else
      typecheck(options.output_qdd,'logical');
    end

    if options.output_qdd
      output_frame = MultiCoordinateFrame({r.getInputFrame(),qddframe});
    else
      output_frame = r.getInputFrame();
    end

    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.numq = getNumPositions(r);
    obj.numv = getNumVelocities(r);
    obj.controller_data = controller_data;
    obj.n_body_accel_inputs = length(body_accel_input_frames);

    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.001;
    end
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate

    if isfield(options,'use_bullet')
      obj.use_bullet = options.use_bullet;
    else
      obj.use_bullet = false;
    end

    % weight for the hdot objective term
    if isfield(options,'W_kdot')
      typecheck(options.W_kdot,'double');
      sizecheck(options.W_kdot,[3 3]);
      obj.W_kdot = options.W_kdot;
    else
      obj.W_kdot = zeros(3);
    end

    % weight for the desired qddot objective term
    if isfield(options,'w_qdd')
      typecheck(options.w_qdd,'double');
      sizecheck(options.w_qdd,[obj.numq 1]); % assume diagonal cost
      obj.w_qdd = options.w_qdd;
    else
      obj.w_qdd = 0.1*ones(obj.numq,1);
    end

    % weight for grf coefficients
    if isfield(options,'w_grf')
      typecheck(options.w_grf,'double');
      sizecheck(options.w_grf,1);
      obj.w_grf = options.w_grf;
    else
      obj.w_grf = 0.0;
    end

    % weight for slack vars
    if isfield(options,'w_slack')
      typecheck(options.w_slack,'double');
      sizecheck(options.w_slack,1);
      obj.w_slack = options.w_slack;
    else
      obj.w_slack = 0.001;
    end

    % proportunal gain for angular momentum
    if isfield(options,'Kp_ang')
      typecheck(options.Kp_ang,'double');
      sizecheck(options.Kp_ang,1);
      obj.Kp_ang = options.Kp_ang;
    else
      obj.Kp_ang = 1.0;
    end

    % gain for support acceleration constraint: accel=-Kp_accel*vel
    if isfield(options,'Kp_accel')
      typecheck(options.Kp_accel,'double');
      sizecheck(options.Kp_accel,1);
      obj.Kp_accel = options.Kp_accel;
    else
      obj.Kp_accel = 0.0; % default desired acceleration=0
    end

    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 10;
    end

    % array dictating whether body acceleration inputs should be
    % constraints (val<0) or cost terms with weight in [0,inf]
    if isfield(options,'body_accel_input_weights')
      typecheck(options.body_accel_input_weights,'double');
      sizecheck(options.body_accel_input_weights,obj.n_body_accel_inputs);
      obj.body_accel_input_weights = options.body_accel_input_weights;
    else
      obj.body_accel_input_weights = -1*ones(obj.n_body_accel_inputs,1);
    end

    % struct array of body acceleration bounds with fields: body_idx,
    % min_acceleration, max_acceleration
    if isfield(options,'body_accel_bounds')
      typecheck(options.body_accel_bounds,'struct');
      obj.body_accel_bounds = options.body_accel_bounds;
      obj.n_body_accel_bounds = length(obj.body_accel_bounds);
    else
      obj.body_accel_bounds = [];
      obj.n_body_accel_bounds = 0;
    end

    if isfield(options,'solver')
      % 0: fastqp, fallback to gurobi barrier (default)
      % 1: gurobi primal simplex with active sets
      typecheck(options.solver,'double');
      sizecheck(options.solver,1);
      assert(options.solver==0 || options.solver==1);
    else
      options.solver = 0;
    end
    obj.solver = options.solver;

    obj.gurobi_options.outputflag = 0; % not verbose
    if options.solver==0
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

    if isa(getTerrain(r),'DRCFlatTerrainMap')
      obj.using_flat_terrain = true;
    else
      obj.using_flat_terrain = false;
    end

    [obj.jlmin, obj.jlmax] = getJointLimits(r);
        
    obj.output_qdd = options.output_qdd;
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
      contact_pts{ind} = plan_supp.contact_pts{lfoot_plan_supp_ind};
      contact_groups{ind} = plan_supp.contact_groups{lfoot_plan_supp_ind};
      n_contact_pts(ind) = plan_supp.num_contact_pts(lfoot_plan_supp_ind);
      ind=ind+1;
    end
    if fc(2)>0
      support_bodies(ind) = r.foot_body_id.right;
      contact_pts{ind} = plan_supp.contact_pts{rfoot_plan_supp_ind};
      contact_groups{ind} = plan_supp.contact_groups{rfoot_plan_supp_ind};
      n_contact_pts(ind) = plan_supp.num_contact_pts(rfoot_plan_supp_ind);
    end
    
    supp.bodies = support_bodies;
    supp.contact_pts = contact_pts;
    supp.contact_groups = contact_groups;
    supp.num_contact_pts = n_contact_pts;
    
    qdd_lb =-500*ones(1,nq);
    qdd_ub = 500*ones(1,nq);
    w_qdd = obj.w_qdd;

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

    include_angular_momentum = any(any(obj.W_kdot));

    if include_angular_momentum
      A = centroidalMomentumMatrix(r, kinsol);
      Adot_times_v = centroidalMomentumMatrixDotTimesVmex(r, kinsol.mex_ptr);
    end

    Jcomdot_times_v = centerOfMassJacobianDotTimesV(r, kinsol);
    if length(x0)==4
     Jcom = Jcom(1:2,:); % only need COM x-y
     Jcomdot_times_v = Jcomdot_times_v(1:2);
    end

    if ~isempty(active_supports)
      nc = sum(num_active_contacts);
      c_pre = 0;
      Dbar = [];
      for j=1:length(active_supports)
        [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet,...
          'body_idx',[1,active_supports(j)],'collision_groups',active_contact_groups(j)));
        Dbar = [Dbar, vertcat(JB{:})'];
        c_pre = c_pre + length(active_contact_pts{j});
      end

      Dbar_float = Dbar(float_idx,:);
      Dbar_act = Dbar(act_idx,:);

      terrain_pts = getTerrainContactPoints(r,active_supports,active_contact_groups);
      [~,Jp] = terrainContactPositions(r,kinsol,terrain_pts,true);
      Jpdot_times_v = terrainContactJacobianDotTimesV(r, kinsol, terrain_pts);
      Jp = sparse(Jp);

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
      Jbdot_times_v = forwardJacDotTimesV(r,kinsol,body_idx,[0;0;0],1);
      Ain_{constraint_index} = Jb*Iqdd;
      bin_{constraint_index} = -Jbdot_times_v + obj.body_accel_bounds(ii).max_acceleration;
      constraint_index = constraint_index + 1;
      Ain_{constraint_index} = -Jb*Iqdd;
      bin_{constraint_index} = Jbdot_times_v - obj.body_accel_bounds(ii).min_acceleration;
      constraint_index = constraint_index + 1;
    end

    if nc > 0
      % relative acceleration constraint
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdot_times_v - obj.Kp_accel*Jp*qd;
    end

    eq_count=3;

    for ii=1:obj.n_body_accel_inputs
      if obj.body_accel_input_weights(ii) < 0
        body_input = varargin{ii+3};
        body_ind = body_input(1);
        body_vdot = body_input(2:7);
        if ~any(active_supports==body_ind)
          [~,J] = forwardKin(r,kinsol,body_ind,[0;0;0],1);
          Jdot_times_v = forwardJacDotTimesV(r, kinsol, body_ind, [0;0;0], 1);
          cidx = ~isnan(body_vdot);
          Aeq_{eq_count} = J(cidx,:)*Iqdd;
          beq_{eq_count} = -Jdot_times_v(cidx) + body_vdot(cidx);
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
      fqp = fqp + Jcomdot_times_v'*R_DQyD_ls*Jcom*Iqdd;
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
          Jdot_times_v = forwardJacDotTimesV(r.getManipulator(),kinsol,body_ind,[0;0;0],1);
          cidx = ~isnan(body_vdot);
          Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + w*J(cidx,:)'*J(cidx,:);
          fqp = fqp + w*(Jdot_times_v(cidx,:)'- body_vdot(cidx)')*J(cidx,:)*Iqdd;
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

    % % compute V,Vdot
    % if (nc>0)
    %   %V = x_bar'*S*x_bar + s1'*x_bar + s2;
    %   %Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*Sdot*x_bar + x_bar'*s1dot + s2dot;
    %   % note for ZMP dynamics, S is constant so Sdot=0
    %   Vdot = (2*x_bar'*S + s1')*(A_ls*x_bar + B_ls*(Jdot*qd + J*qdd)) + x_bar'*s1dot + s2dot;
    % end

    if obj.output_qdd
      varargout = {y,qdd};
    else
      varargout = {y};
    end
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    numv;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    W_kdot; % angular momentum cost term weight matrix
    w_qdd; % qdd objective function weight vector
    w_grf; % scalar ground reaction force weight
    w_slack; % scalar slack var weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    Kp_ang; % proportunal gain for angular momentum feedback
    Kp_accel; % gain for support acceleration constraint: accel=-Kp_accel*vel
    gurobi_options = struct();
    solver=0;
    lc;
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    use_bullet;
    using_flat_terrain; % true if using DRCFlatTerrain
    jlmin;
    jlmax;
    output_qdd = false;
    body_accel_input_weights; % array of doubles, negative values signal constraints
    n_body_accel_inputs;
    body_accel_bounds;
    n_body_accel_bounds;
  end
end
