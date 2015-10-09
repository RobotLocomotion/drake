classdef FullStateQPControllerDT < DrakeSystem
  methods
  function obj = FullStateQPControllerDT(r,controller_data,options)
    % @param r rigid body manipulator instance
    % @param controller_data FullStateQPControllerData object containing the matrices that
    % @param options structure for specifying objective weights, slack
    % bounds, etc.
    typecheck(r,'TimeSteppingRigidBodyManipulator');
    typecheck(controller_data,'FullStateQPControllerData');
    
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
            
    input_frame = r.getStateFrame();
    output_frame = r.getInputFrame();
    
    obj = obj@DrakeSystem(0,0,input_frame.dim,output_frame.dim,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.numq = getNumPositions(r);
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

    % weight for cpos slack vars
    if isfield(options,'w_cpos_slack')
      typecheck(options.w_cpos_slack,'double');
      sizecheck(options.w_cpos_slack,1);
      obj.w_cpos_slack = options.w_cpos_slack;
    else
      obj.w_cpos_slack = 0.001;
    end       
    
    % weight for phi slack vars
    if isfield(options,'w_phi_slack')
      typecheck(options.w_phi_slack,'double');
      sizecheck(options.w_phi_slack,1);
      obj.w_phi_slack = options.w_phi_slack;
    else
      obj.w_phi_slack = 0.001;
    end       

    % gain for support acceleration constraint: accel=-Kp_accel*vel
    if isfield(options,'Kp_accel')
      typecheck(options.Kp_accel,'double');
      sizecheck(options.Kp_accel,1);
      obj.Kp_accel = options.Kp_accel;
    else
      obj.Kp_accel = 0.0; % default desired acceleration=0
    end       
    
    if isfield(options,'Kp_phi')
      typecheck(options.Kp_phi,'double');
      sizecheck(options.Kp_phi,1);
      obj.Kp_phi = options.Kp_phi;
    else
      obj.Kp_phi = 10;
    end

    if isfield(options,'Kd_phi')
      typecheck(options.Kd_phi,'double');
      sizecheck(options.Kd_phi,1);
      obj.Kd_phi = options.Kd_phi;
    else
      obj.Kd_phi = 2*sqrt(obj.Kp_phi);
    end

    % hard bound on cpos_ddot slack variables
    if isfield(options,'cpos_slack_limit')
      typecheck(options.cpos_slack_limit,'double');
      sizecheck(options.cpos_slack_limit,1);
      obj.cpos_slack_limit = options.cpos_slack_limit;
    else
      obj.cpos_slack_limit = 10;
    end

    % hard bound on phi_ddot slack variables
    if isfield(options,'phi_slack_limit')
      typecheck(options.phi_slack_limit,'double');
      sizecheck(options.phi_slack_limit,1);
      obj.phi_slack_limit = options.phi_slack_limit;
    else
      obj.phi_slack_limit = 10;
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
    
    if isfield(options,'offset_x')
      typecheck(options.offset_x,'logical');
      obj.offset_x = options.offset_x;
    else
      obj.offset_x = true;
    end
    
    if isfield(options,'contact_threshold')
      % minimum height above terrain for points to be in contact
      typecheck(options.contact_threshold,'double');
      sizecheck(options.contact_threshold,[1 1]);
      obj.contact_threshold = options.contact_threshold;
    else
      obj.contact_threshold = 0.001;
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
    
    
    % generate constrained optimnizations
    contact_q0 = zeros(r.getNumPositions,1);
    for i=1:length(controller_data.supports)      
      to_options.non_penetration = false;
      to_options.collocation_friction_limits = false;
      to_options.constrain_start = false;
      hyb_to_options.mode_options = {to_options,to_options};
      
      mode_indices = repmat(controller_data.contact_seq{i},1,2);
      obj.constrained_trajopts(i) = ConstrainedHybridTrajectoryOptimization(r.getManipulator,mode_indices,{2,1},{[-inf inf], [-inf inf]},hyb_to_options);      
%       obj.constrained_trajopts(i) = ContactConstrainedDircolTrajectoryOptimization(r.getManipulator,2,[-inf inf],controller_data.contact_seq{i},to_options);      
    end
  end
    
  function y=output(obj,t,~,x)
    global utraj_pts utraj_ts
    nu = getNumInputs(r);
    if isempty(utraj_pts)
      utraj_pts = []; 
      utraj_ts = [];
    end
    ctrl_data = obj.controller_data;
    
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 

    kinsol = doKinematics(r,q,true,true,qd);
    
    supp_idx = find(ctrl_data.support_times<=t,1,'last');
    next_supp_idx = min(supp_idx+1,length(ctrl_data.support_times));
    
    
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
      if isa(ctrl_data.x0,'Trajectory')
        x0 = fasteval(ctrl_data.x0,t);
      else
        x0 = fasteval(ctrl_data.x0{supp_idx},t);
      end
      if isa(ctrl_data.u0,'Trajectory')
        u0 = fasteval(ctrl_data.u0,t);
      else
        u0 = fasteval(ctrl_data.u0{supp_idx},t);
      end
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
    end
    q0 = x0(1:nq);
    
    %% New controller code here
    h = obj.getSampleTime;
    u_last = utraj_pts(:,end);
    traj_opt = obj.constrained_trajopts(supp_idx);
    x_guess = x; % maybe use nominal?
    u_guess = u_last; % maybe use nominal?
    z0 = zeros(traj_opt.num_vars,1);
    z0(traj_opt.mode_opt{1}.h_inds) = h;
    z0(traj_opt.mode_opt{1}.x_inds) = [x x_guess];
    z0(traj_opt.mode_opt{1}.u_inds) = [u_last u_guess];
    
    % Possible things to try
    % 1) Linearizae about x or xnom
    % 2) Actually use FOH controller
    % 3) Handle "late" contacts
    %    a) Extract contact state
    %    b) Zero forces for states not in contact
    %    c) Impulse event
    % 4) Handle "early" contacts
    %    a) Allow an early switch to next-mode
    
    % Linearize Constraints
    [~,~,G_in,G_eq] = traj_opt.nonlinearConstraints(z0);
    A_in = [traj_opt.Ain;-G_in(~isinf(traj_opt.cin_lb),:);G_in(~isinf(traj_opt.cin_ub),:)];
    b_in = [traj_opt.bin;-traj_opt.cin_lb(~isinf(obj.cin_lb));traj_opt.cin_ub(~isinf(obj.cin_ub))];
    A_eq = [traj_opt.Aeq;Geq];
    b_eq = [traj_opt.beq;zeros(size(Geq,1),1)];
    
    % Cost
    % (x-x0)^T*S*(x-x0) + h*(u-u0)^T*R*(u-u0)
    x_ind = traj_opt.mode_opt{1}.x_inds(:,2);
    u_ind = traj_opt.mode_opt{1}.u_inds(:,2);
    H = zeros(traj_opt.num_vars);
    H(x_ind,x_ind) = S;
    H(u_ind,u_ind) = h*R;
    
    f = zeros(traj_opt.num_vars,1);
    f(x_ind) = -2*S*x0;
    f(u_ind) = -2*R*u0;
    
    lb = traj_opt.x_lb;
    ub = traj_opt.x_ub;
    
    lb(traj_opt.mode_opt{1}.x_inds(:,1)) = x;
    ub(traj_opt.mode_opt{1}.x_inds(:,1)) = x;
    u_option = 1;
    if u_option == 1,
      % Assume u0 fixed, solve for u1      
      lb(traj_opt.mode_opt{1}.u_inds(:,1)) = u_last;
      ub(traj_opt.mode_opt{1}.u_inds(:,1)) = u_last;
    else
      % Assume u0 = u1
      A_sub = zeros(nu,traj_opt.num_vars);
      A_sub(:,traj_opt.mode_opt{1}.u_inds(:,1)) = eye(nu);
      A_sub(:,traj_opt.mode_opt{1}.u_inds(:,2)) = -eye(nu);
      A_eq = [A_eq;A_sub];
      b_eq = [b_eq;zeros(nu,1)];
    end
    
    %TODO: eliminate unnecessary variables. Fixed, or absent in gradient
    
    [z,fval,exitflag] = quadprog(H,f,A_in,b_in,A_eq,b_eq,lb,ub,z0);
    
    if exitflag ~= 1
      keyboard
    end
    
    y = z(u_ind);
    return
    %% OLD CODE IS BELOW HERE
    test_next_support = false;
    if next_supp_idx > supp_idx && ctrl_data.support_times(next_supp_idx)-t < 0.025
      test_next_support = true;
    end
    
    if test_next_support
      cur_rigid_body_support_state = ctrl_data.supports(supp_idx);
      try
      rigid_body_support_state = ctrl_data.supports(next_supp_idx);
      catch
        keyboard
      end
      if obj.hasImpact(cur_rigid_body_support_state,rigid_body_support_state)
        planned_supports = rigid_body_support_state.bodies;
        planned_contact_groups = rigid_body_support_state.contact_groups;

        phi = [];
        np = 0;
        for j=1:length(planned_supports)
          phi_j = contactConstraints(r,kinsol,false,struct('terrain_only',1,...
            'body_idx',[1,planned_supports(j)],'collision_groups',planned_contact_groups(j)));
          phi = [phi; phi_j];
        end
        if all(phi <= 1e-4)
          supp_idx = next_supp_idx;
          t=ctrl_data.support_times(supp_idx);
          disp('SWITCHING TO THE NEXT MODE!');
        end
      end
    end
    
 
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
      if isa(ctrl_data.x0,'Trajectory')
        x0 = fasteval(ctrl_data.x0,t);
      else
        x0 = fasteval(ctrl_data.x0{supp_idx},t);
      end
      if isa(ctrl_data.u0,'Trajectory')
        u0 = fasteval(ctrl_data.u0,t);
      else
        u0 = fasteval(ctrl_data.u0{supp_idx},t);
      end
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
    end
    q0 = x0(1:nq);
    
    % get phi for planned contact groups
    % if phi < threshold, then add to active contacts
    % else, add to desired contacts
    
    if max(supp_idx) > length(ctrl_data.supports)
      keyboard
    end

    rigid_body_support_state = ctrl_data.supports(supp_idx);
    
    all_supports = ctrl_data.allowable_supports.bodies;
    all_contact_groups = ctrl_data.allowable_supports.contact_groups;

    planned_supports = rigid_body_support_state.bodies;
    planned_contact_groups = rigid_body_support_state.contact_groups;
    %planned_num_contacts = rigid_body_support_state.num_contact_pts;      

    dim = obj.robot.getManipulator.dim; % 2D or 3D

    Jn = [];
    Jndot = [];
    phi_err = [];
    Dbar = [];
    xp = [];
    Jp = [];
    Jpdotv = [];
    nc = 0;
    
    % ridiculously inefficient for testing
    for j=1:length(all_supports)
      [phi,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',1,...
          'body_idx',[1,all_supports(j)],'collision_groups',all_contact_groups(j)));
      
      active_ind = phi<=obj.contact_threshold;
      nc = nc+sum(active_ind);
      Dbar = [Dbar, vertcat(JB{active_ind})']; 
    end

    np = 0;
    leading_x_pos = -inf;
    for j=1:length(planned_supports)
          kinsol = doKinematics(r,q,true,true,qd);

      [phi,~,~,~,~,~,~,~,n,~,dn,~] = contactConstraints(r,kinsol,false,struct('terrain_only',1,...
          'body_idx',[1,planned_supports(j)],'collision_groups',planned_contact_groups(j)));
      
      active_ind = phi<=obj.contact_threshold*5;
      phi_err = [phi_err;-phi(~active_ind)];

      np = np+sum(active_ind);
      ndot = matGradMult(dn,qd);
      Jn = [Jn; n(~active_ind,:)];
      Jndot = [Jndot; ndot(~active_ind,:)];

       % hacky here because we're lacking planar system support
      terrain_pts = getTerrainContactPoints(r,planned_supports(j),planned_contact_groups(j));
      pts = [terrain_pts.pts];
      pts = pts(:,active_ind);

      if obj.robot.getManipulator.dim == 2
        xz_pts = pts([1 3],:);
      else
        xz_pts = pts;
      end
      
      [xp_j,Jp_j] = forwardKin(r,kinsol,planned_supports(j),xz_pts,0);
      xp_jrot = forwardKin(r,kinsol,planned_supports(j),xz_pts,1);
      Jpdotv_j = r.getManipulator.forwardJacDotTimesV(kinsol,planned_supports(j),pts,0);
          
      xp = [xp,xp_j];
      Jp = [Jp;Jp_j];
      Jpdotv = [Jpdotv;Jpdotv_j];

      if exist('xz_pts') && ~isempty(xz_pts)
        % compute foot placement error
        kinsol0 = r.doKinematics(q0);
        xp0 = forwardKin(r,kinsol0,planned_supports(j),xz_pts,1);
        if any(xp_j(1,:) > leading_x_pos)
          leading_x_pos = max(xp_j(1,:));
          obj.controller_data.xoffset = (mean(xp_j(1,:)-xp0(1,:))); 
%           yoffset = (mean(xp_j(2,:)-xp0(2,:)));
%           yawoffset = (mean(xp_jrot(6,:)-xp0(6,:)));
        end

      end


    end

    
    if dim==2
       % delete y rows
      yind = 2:3:np*3;
      Jpdotv(yind,:) = [];
      Jp = sparse(Jp);
      Jpdotv = sparse(Jpdotv);

      nd = 2; % for friction cone approx, hard coded for now
    elseif dim==3
      nd = 4; % for friction cone approx, hard coded for now
    end
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    neps = np*dim;
    if obj.offset_x
      if r.getManipulator.dim == 3
        xoffset = obj.controller_data.xoffset;
        x0(1) = x0(1) + obj.controller_data.xoffset;
%         x0(2) = x0(2) + yoffset;
%         x0(6) = x0(6) + yawoffset;
      else
        xoffset = obj.controller_data.xoffset;
        x0(1) = x0(1) + obj.controller_data.xoffset;
      end
    end
    display(sprintf('t: %.3f S: %3.3e',t,(x-x0)'*S*(x-x0)));
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    nu = getNumInputs(r);
    nf = nc*nd; % number of contact force variables
    nparams = nu+2*nq+4*nf;
    Iu = zeros(nu,nparams); Iu(:,1:nu) = eye(nu);
    Ix = zeros(2*nq,nparams); Iqdd(:,nu+(1:2*nq)) = eye(2*nq);
    Iq = Ix(1:nq,:);
    Iqd = Ix(nq+1:end,:);    
    Ibeta0 = zeros(nf,nparams); Ibeta(:,nu+2*nq+(1:nf)) = eye(nf);
    Ibeta1 = zeros(nf,nparams); Ibeta(:,nu+2*nq+nf+(1:nf)) = eye(nf);
    Ibetac = zeros(nf,nparams); Ibeta(:,nu+2*nq+2*nf+(1:nf)) = eye(nf);
    Ivc = zeros(nf,nparams); Ibeta(:,nu+2*nq+3*nf+(1:nf)) = eye(nf);

    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [r.umin' -inf*ones(1,2*nq) zeros(1,4*nf)]';
    ub = [r.umax' inf*ones(1,2*nq) inf(1,4*nf)]';

    Aeq_ = {};
    beq_ = {};

    % dynamics constraints
    
    
    if nc>0
      Aeq_{1} = H*Iqdd - B*Iu - Dbar*Ibeta;
    else
      Aeq_{1} = H*Iqdd - B*Iu;
    end
    beq_{1} = -C;

    if np > 0
      % relative acceleration constraint
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdotv - obj.Kp_accel*Jp*qd; 
    end

%     if ~isempty(phi_err)
%       phi_ddot_desired = obj.Kp_phi*phi_err - obj.Kd_phi*(Jndot*qd);
%       Aeq_{3} = Jn*Iqdd + Ieta;
%       beq_{3} = -Jndot*qd + phi_ddot_desired; 
%     end
    
    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});
    
%     phi = contactConstraints(r,x0(1:nq),false,struct('terrain_only',1));
%     active_constraints = ctrl_data.mode_data{supp_idx}.constraint_ind;
%     dynamicsFun = @(t,x,u) constrainedDynamics(r,t,x0,u,active_constraints);
%     [~,dxd] = geval(dynamicsFun,t,x0,u0,struct('grad_method','numerical'));
%     B_ls = dxd(:,getNumStates(r)+1+(1:nu));
    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    %
    % min: ubar*R*ubar + 2*xbar'*S*B*u + w_eps*quad(epsilon) + w_grf*quad(beta) 
    xbar = x-x0; 
    Hqp = Iu'*R*Iu;
    fqp = xbar'*S*B_ls*Iu;
    fqp = fqp-u0'*R*Iu;

    Hqp(nu+(1:nq),nu+(1:nq)) = diag(obj.w_qdd);
    if nc > 0
      Hqp(nu+nq+(1:nf),nu+nq+(1:nf)) = obj.w_grf*eye(nf); 
      Hqp(nu+nq+nf+(1:neps),nu+nq+nf+(1:neps)) = obj.w_cpos_slack*eye(neps); 
      Hqp(nu+nq+nf+neps+(1:neta),nu+nq+nf+neps+(1:neta)) = obj.w_phi_slack*eye(neta); 
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
                obj.w_cpos_slack*ones(neps,1) + REG*ones(neps,1), ...
                obj.w_phi_slack*ones(neta,1) + REG*ones(neta,1)};
              
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

      result = gurobi(model,obj.gurobi_options);
      alpha = result.x;

      qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
      obj.controller_data.qp_active_set = qp_active_set;
    end
    
%     beta=Ibeta*alpha
    y = Iu*alpha;
%     y = u0;

    nc_np = [nc,np];

    utraj_pts = [utraj_pts y];
    utraj_ts = [utraj_ts t];
    if any(alpha-lb == 0) || any(ub-alpha==0)
      blah=2;
    end

  end
 

  function bool=hasImpact(~,rb_support_state1, rb_support_state2)
    bool=true;
    
    bodies1 = rb_support_state1.bodies;
    bodies2 = rb_support_state2.bodies;
    groups1 = rb_support_state1.contact_groups;
    groups2 = rb_support_state2.contact_groups;
    
    if isempty(bodies2)
      bool=false; % no impacts
      return
    end
    bodies_union = union(bodies1,bodies2);
    if length(intersect(bodies1,bodies_union))<length(bodies_union)
      % new body coming into contact
      return
    end
    
    for i=1:length(bodies2)
      group1_i = groups1{bodies1 == bodies2(i)};
      num_groups1 = length(group1_i);
      num_groups2 = length(groups2{i});
      if num_groups2 > num_groups1 || length(intersect(group1_i,groups2{i})) < num_groups2
        return
      end
    end
    bool=false;
  end
    
  end

  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w_grf; % scalar ground reaction force weight
    w_qdd; % qdd objective function weight vector
    w_cpos_slack; % scalar slack var weight
    w_phi_slack; % scalar slack var weight
    cpos_slack_limit; 
    phi_slack_limit; 
    Kp_accel; % gain for support acceleration constraint: accel=-Kp_accel*vel
    Kp_phi; 
    Kd_phi; 
    gurobi_options = struct();
    solver=0;
    lc;
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    jlmin;
    jlmax;
    contact_threshold;
    offset_x; % whether or not to offset the nominal state in the x-dimension
    constrained_trajopts
  end
end
