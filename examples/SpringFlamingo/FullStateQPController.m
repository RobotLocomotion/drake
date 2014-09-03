classdef FullStateQPController < MIMODrakeSystem
  methods
  function obj = FullStateQPController(r,controller_data,options)
    % @param r rigid body manipulator instance
    % @param controller_data FullStateQPControllerData object containing the matrices that
    % define the underlying linear system, the ZMP trajectory, Riccati
    % solution, etc.
    % @param options structure for specifying objective weights, slack
    % bounds, etc.
    typecheck(r,'TimeSteppingRigidBodyManipulator');
    typecheck(controller_data,'FullStateQPControllerData');
    
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
            
    input_frame = MultiCoordinateFrame({r.getStateFrame,FootContactState});
    output_frame = r.getInputFrame();
    
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.numq = getNumDOF(r);
    obj.controller_data = controller_data;
    
    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.001;
    end
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate
   
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
    
    if isfield(options,'left_foot_name')
      obj.lfoot_idx = findLinkInd(r,options.left_foot_name);
    else
      obj.lfoot_idx = findLinkInd(r,'left_foot');
    end

    if isfield(options,'right_foot_name')
      obj.rfoot_idx = findLinkInd(r,options.right_foot_name);
    else
      obj.rfoot_idx = findLinkInd(r,'right_foot');
    end
      
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
            
    [obj.jlmin, obj.jlmax] = getJointLimits(r);
  end
    
  function y=mimoOutput(obj,t,~,varargin)
    %out_tic = tic;

    ctrl_data = obj.controller_data;
      
    x = varargin{1};
    fc = varargin{2};
       
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 
            
    supp_idx = find(ctrl_data.support_times<=t,1,'last');

    if ctrl_data.B_is_time_varying
      if isa(ctrl_data.B,'Trajectory')
        B_ls = fasteval(ctrl_data.B,t);
      else
        B_ls = fasteval(ctrl_data.B{supp_idx},t);
      end
    else
      B_ls = ctrl_data.B; 
    end
    R = ctrl_data.R;
    if (ctrl_data.lqr_is_time_varying)
      if isa(ctrl_data.S,'Trajectory')
        S = fasteval(ctrl_data.S,t);
      else
        S = fasteval(ctrl_data.S{supp_idx},t);
      end
      x0 = fasteval(ctrl_data.x0,t);
      u0 = fasteval(ctrl_data.u0,t);
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
    end
    q0 = x0(1:nq);

    support_bodies = [];
    contact_pts = {};
    contact_groups = {};
    n_contact_pts = [];
    ind = 1;
    
    plan_supp = ctrl_data.supports(supp_idx);

    lfoot_plan_supp_ind = plan_supp.bodies==obj.lfoot_idx;
    rfoot_plan_supp_ind = plan_supp.bodies==obj.rfoot_idx;
    if fc(1)>0
      support_bodies(ind) = obj.lfoot_idx;
      contact_pts{ind} = plan_supp.contact_pts{lfoot_plan_supp_ind};
      contact_groups{ind} = plan_supp.contact_groups{lfoot_plan_supp_ind};
      n_contact_pts(ind) = plan_supp.num_contact_pts(lfoot_plan_supp_ind);
      ind=ind+1;
    end
    if fc(2)>0
      support_bodies(ind) = obj.rfoot_idx;
      contact_pts{ind} = plan_supp.contact_pts{rfoot_plan_supp_ind};
      contact_groups{ind} = plan_supp.contact_groups{rfoot_plan_supp_ind};
      n_contact_pts(ind) = plan_supp.num_contact_pts(rfoot_plan_supp_ind);
    end
    
    supp.bodies = support_bodies;
    supp.contact_pts = contact_pts;
    supp.contact_groups = contact_groups;
    supp.num_contact_pts = n_contact_pts;
    supp.contact_surfaces = 0*support_bodies;
    
    kinsol = doKinematics(r,q,false,true,qd);

    active_supports = supp.bodies;
    active_contact_pts = supp.contact_pts;
    active_contact_groups = supp.contact_groups;
    num_active_contacts = supp.num_contact_pts;      

    dim = 2; % 3D
    if dim==2
      nd = 2; % for friction cone approx, hard coded for now
    elseif dim==3
      nd = 4; % for friction cone approx, hard coded for now
    end
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    if ~isempty(active_supports)
      nc = sum(num_active_contacts);
      c_pre = 0;
      Dbar = [];
      for j=1:length(active_supports)
        [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',1,...
          'body_idx',[1,active_supports(j)],'collision_groups',active_contact_groups(j)));
        Dbar = [Dbar, vertcat(JB{active_contact_pts{j}})']; % because contact constraints seems to ignore the collision_groups option
        c_pre = c_pre + length(active_contact_pts{j});
      end
      
      % hacky here because we're lacking planar system support
      terrain_pts = getTerrainContactPoints(r,active_supports,active_contact_groups);
      pts = [terrain_pts.pts];
      xz_pts = pts([1 3],:);
      [xp,Jp] = forwardKin(r,kinsol,active_supports,xz_pts,0);
      Jpdot = forwardJacDot(r,kinsol,active_supports,pts,0);
     
      % compute foor placement error
      kinsol0 = r.doKinematics(q0);
      xp0 = forwardKin(r,kinsol0,active_supports,xz_pts,0);
      xoffset = mean(xp(1,:) - xp0(1,:))
      x0(1) = x0(1) - xoffset;
      
      % delete y rows
      yind = 2:3:nc*3;
      Jpdot(yind,:) = [];
      Jp = sparse(Jp);
      Jpdot = sparse(Jpdot);
    else
      nc = 0;
    end
    neps = nc*dim;


    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    nu = getNumInputs(r);
    nf = nc*nd; % number of contact force variables
    nparams = nu+nq+nf+neps;
    Iu = zeros(nu,nparams); Iu(:,1:nu) = eye(nu);
    Iqdd = zeros(nq,nparams); Iqdd(:,nu+(1:nq)) = eye(nq);
    Ibeta = zeros(nf,nparams); Ibeta(:,nu+nq+(1:nf)) = eye(nf);
    Ieps = zeros(neps,nparams);
    Ieps(:,nu+nq+nf+(1:neps)) = eye(neps);


    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [r.umin' -inf*ones(1,nq) zeros(1,nf)   -obj.slack_limit*ones(1,neps)]'; % qddot/contact forces/slack vars
    ub = [r.umax' inf*ones(1,nq) inf*ones(1,nf) obj.slack_limit*ones(1,neps)]';

    Aeq_ = cell(1,2);
    beq_ = cell(1,2);

    % constrained dynamics
    if nc>0
      Aeq_{1} = H*Iqdd - B*Iu - Dbar*Ibeta;
    else
      Aeq_{1} = H*Iqdd - B*Iu;
    end
    beq_{1} = -C;

    if nc > 0
      % relative acceleration constraint
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdot*qd - obj.Kp_accel*Jp*qd; 
    end

    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});
    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    %
    % min: ubar*R*ubar + 2*xbar'*S*B*u + w_eps*quad(epsilon) + w_grf*quad(beta) 
    xbar = x-x0; 
    Hqp = Iu'*R*Iu;
      
    fqp = xbar'*S*B_ls*Iu;
%       Kp = 1; Kd = 1.0*sqrt(Kp);
%       qdd_des = Kp*(x0(1:nq)-x(1:nq)) + Kd*(x0(nq+(1:nq))-x(nq+(1:nq)))
%       fqp = -qdd_des'*Q*Iqdd;
    fqp = fqp - u0'*R*Iu;
%       fqp = -u0'*R*Iu;

    Hqp(nu+(1:nq),nu+(1:nq)) = diag(obj.w_qdd);
    if nc > 0
      Hqp(nu+nq+(1:nf),nu+nq+(1:nf)) = obj.w_grf*eye(nf); 
      Hqp(nparams-neps+1:end,nparams-neps+1:end) = obj.w_slack*eye(neps); 
    end
    
    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------

    REG = 1e-8;

    IR = eye(nparams);  
    lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
    Ain_fqp = full([-IR(lbind,:); IR(ubind,:)]);
    bin_fqp = [-lb(lbind); ub(ubind)];

    % call fastQPmex first
    QblkDiag = {Hqp(1:(nu+nq),1:(nu+nq)) + REG*eye(nu+nq), ...
                obj.w_grf*ones(nf,1) + REG*ones(nf,1), ...
                obj.w_slack*ones(neps,1) + REG*ones(neps,1)};
    Aeq_fqp = full(Aeq);
    % NOTE: model.obj is 2* f for fastQP!!!
    [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

    if info_fqp<0
      % then call gurobi
      disp('QPController: failed over to gurobi');
      model.Q = sparse(Hqp + REG*eye(nparams));
      model.A = Aeq;
      model.rhs = beq;
      model.sense = obj.eq_array(1:length(beq));
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

      qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
      obj.controller_data.qp_active_set = qp_active_set;
    end
    beta=Ibeta*alpha
    y = Iu*alpha     
    
  end
  end

  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w_grf; % scalar ground reaction force weight
    w_qdd; % qdd objective function weight vector
    w_slack; % scalar slack var weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    Kp_accel; % gain for support acceleration constraint: accel=-Kp_accel*vel
    rfoot_idx;
    lfoot_idx;
    gurobi_options = struct();
    solver=0;
    lc;
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    jlmin;
    jlmax;
  end
end
