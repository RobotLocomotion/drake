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

    % time-step for control lookahead
    if isfield(options,'timestep')
      typecheck(options.timestep,'double');
      sizecheck(options.timestep,1);
      obj.timestep = options.timestep;
    else
      obj.timestep = 0.001;
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
      obj.gurobi_options.bariterlimit = 2000; % iteration limit
      obj.gurobi_options.barhomogeneous = 0; % 0 off, 1 on
      obj.gurobi_options.barconvtol = 5e-4;
    end
            
    [obj.jlmin, obj.jlmax] = getJointLimits(r);
  end
  
  
  function y=output_nlp(obj,t,~,x)
    ctrl_data = obj.controller_data;
      
    r = obj.robot;
    nq = obj.numq; 
    nu = getNumInputs(r);
    q = x(1:nq); 
    qd = x(nq+(1:nq));             

    h=obj.timestep;
    
    supp_idx = find(ctrl_data.support_times<=t,1,'last');

%     if ctrl_data.B_is_time_varying
%       if isa(ctrl_data.B,'Trajectory')
%         B_ls = fasteval(ctrl_data.B,t);
%       else
%         B_ls = fasteval(ctrl_data.B{supp_idx},t);
%       end
%     else
%       B_ls = ctrl_data.B; 
%     end
    R = ctrl_data.R;
    if (ctrl_data.lqr_is_time_varying)
      if isa(ctrl_data.S,'Trajectory')
        S = fasteval(ctrl_data.S,t+h);
      else
        S = fasteval(ctrl_data.S{supp_idx},t+h);
      end
      x0 = fasteval(ctrl_data.x0,t+h);
      u0 = fasteval(ctrl_data.u0,t);
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
    end
    q0 = x0(1:nq);
    
    to_options.integration_method = ContactImplicitTrajectoryOptimization.MIXED;
    prog = ContactImplicitTrajectoryOptimization(r,2,[h h],to_options);
    prog = prog.addStateConstraint(BoundingBoxConstraint(x,x),1);
    
    Hqp = [S zeros(2*nq,nu);zeros(nu,2*nq) h*R];
    fqp = [-S*x0; -h*R*u0];
    
    prog = prog.addCost(QuadraticConstraint(0,0,Hqp,fqp),[prog.x_inds(:,end);prog.u_inds(:,1)]);
    
    traj_init.xtraj = PPTrajectory(foh([0 h],[x x0]));
    traj_init.utraj = PPTrajectory(foh([0 h],[u0 u0]));
    
    prog = prog.setSolverOptions('snopt','print','snopt.out');
    prog = prog.setSolverOptions('snopt','MajorIterationsLimit',100);
    prog = prog.setSolverOptions('snopt','MinorIterationsLimit',200000);
    prog = prog.setSolverOptions('snopt','IterationsLimit',200000);
    prog = prog.setSolverOptions('snopt','SuperbasicsLimit',5000);
    prog = prog.setSolverOptions('snopt','MajorOptimalityTolerance',1e-5);
    [xtraj,utraj,ltraj,ljltraj,z,F,info] = prog.solveTraj([0 h],traj_init);
    
  
    y = z(prog.u_inds(:,1));
    
    
    xpred = z(prog.x_inds(:,2));
    x0t = fasteval(ctrl_data.x0,t);
    S0t = ctrl_data.S{supp_idx}.eval(t);
    
    Vpred = (xpred-x0)'*S*(xpred-x0);
    Vcur = (x-x0t)'*S0t*(x-x0t);
    display(sprintf('t=%f, V=%.3f, Vp=%.3f, dV=%.3f',t,Vcur,Vpred,Vpred - Vcur));
    
  end
  
    function y=output_miqp(obj,t,~,x)
%     y = obj.output_nlp(t,[],x);
%     return;
    doProj = false;
    
    
    ctrl_data = obj.controller_data;
      
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 
    h=obj.timestep;
            
    supp_idx = find(ctrl_data.support_times<=(t+h),1,'last');
    support = ctrl_data.supports(supp_idx);
    if support.bodies(1) == 7,
      if length(support.bodies) > 1
        supp_ind = [support.contact_pts{1} support.contact_pts{2}+2];
      else
        supp_ind = support.contact_pts{1};
      end
    else
      supp_ind = support.contact_pts{1} + 2;
    end
    
    

    R = ctrl_data.R;
    if (ctrl_data.lqr_is_time_varying)
      if isa(ctrl_data.S,'Trajectory')
        S = fasteval(ctrl_data.S,t+h);
        S0t = ctrl_data.S.eval(t);
      else
        S = fasteval(ctrl_data.S{supp_idx},t+h);
        S0t = ctrl_data.S{supp_idx}.eval(t);
      end
      x0t = fasteval(ctrl_data.x0,t);
      x0 = fasteval(ctrl_data.x0,t+h);
      u0 = fasteval(ctrl_data.u0,t);
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
      x0t = x0;
    end
    
    % automatic offset for x position
    x_offset = [1;zeros(2*nq-1,1)]'*S0t*(x-x0t)/S0t(1,1);
    x0t(1) = x0t(1) + x_offset;
    x0(1) = x0(1) + x_offset;
    
    q0 = x0(1:nq);

    % get phi for planned contact groups
    % if phi < threshold, then add to active contacts
    % else, add to desired contacts

    kinsol = doKinematics(r,q,true,true,qd);

    dim = 2; % 2D or 3D
    
    [phi,~,~,~,~,~,~,~,n,D] = contactConstraints(r,kinsol,false,struct('terrain_only',1));   
    nc = length(phi);
    nd = length(D);
    D_mat = [D{1};D{2}];
    
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    nu = getNumInputs(r);
    nlambda = nc;
    nlambdaf = nc*nd;
    ngamma = nc;
    
    
    nf = nlambda+nlambdaf+ngamma;
    nbool = nf;
    
    nparams = nu+2*nq+nlambda+nlambdaf+ngamma+nbool;
    Iu = zeros(nu,nparams); Iu(:,1:nu) = eye(nu);
    Iq = zeros(nq,nparams); Iq(:,nu+(1:nq)) = eye(nq);
    Iqd = zeros(nq,nparams); Iqd(:,nu+nq+(1:nq)) = eye(nq);
    Ix = zeros(2*nq,nparams); Ix(:,nu+(1:2*nq)) = eye(2*nq);
    Ilambda = zeros(nlambda,nparams); %Lambda impulse slack vars for projection
    Ilambda(:,nu+2*nq+(1:nlambda)) = eye(nlambda);
    Ilambdaf = zeros(nlambdaf,nparams); %Lambda impulse slack vars for projection
    Ilambdaf(:,nu+2*nq+nlambda+(1:nlambdaf)) = eye(nlambdaf);
    Igamma = zeros(ngamma,nparams);
    Igamma(:,nu+2*nq+nlambda+nlambdaf+(1:ngamma)) = eye(ngamma);
    Ibool = zeros(nbool,nparams);
    Ibool(:,nu+2*nq+nlambda+nlambdaf+ngamma+(1:nbool)) = eye(nbool);
    

    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [(r.umin-00)' obj.jlmin' -inf*ones(1,nq) zeros(1,nf) zeros(1,nbool)]'; % qddot/contact forces/slack vars
    ub = [(r.umax+00)' obj.jlmax' inf*ones(1,nq) inf*ones(1,nf) 1.5*ones(1,nbool)]';

    Aeq_ = cell(1,4);
    beq_ = cell(1,3);

    % dynamics constraints

    Hinv = inv(H);
    h_Hinv = h*Hinv;
    Aeq_{1} = Iqd - h_Hinv*B*Iu - Hinv*n'*Ilambda - Hinv*D_mat'*Ilambdaf;
    beq_{1} = qd - h_Hinv*C;

    Aeq_{2} = Iq - h*Iqd;
    beq_{2} = q;%+h*qd;    
    
    % inequality constraints on phi
    Aineq = -n*Iqd*h;
    bineq = phi;
    
    % LCP constraints
    % z \perp Wz + Mx + w
    % z = [lambda;lambdaf;gamma]
    % x = [q;qd]
    W = zeros(nf,2*nq);
    M = zeros(nf);
    w = zeros(nf,1);
    
    % phi0 + h*phidot \perp lambda
    W(1:nc,nq+1:end) = h*n;
    w(1:nc) = phi;
    
    % gamma + D*qdot \perp lambdaf 
    W(nc+1:2*nc,nq+1:end) = D{1};
    M(nc+1:2*nc,3*nc+1:end) = eye(nc);
    
    W(2*nc+1:3*nc,nq+1:end) = D{2};
    M(2*nc+1:3*nc,3*nc+1:end) = eye(nc);
    
    % mu*lambda - lambdaf \perp gamma
    mu = 1;
    M(3*nc+1:end,:) = [mu*eye(nc) -eye(nc) -eye(nc) zeros(nc)];
    
    
    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});
    
    Iz_lc = [Ilambda;Ilambdaf;Igamma];
    
    BIG = 1000;
    
    Aineq = [Aineq;-Iz_lc;Iz_lc - BIG*Ibool;-M*Iz_lc - W*Ix;M*Iz_lc + W*Ix + BIG*Ibool];
    bineq = [bineq;zeros(2*nf,1);w;-w + BIG];
    
    
    % Generate state guess
    
    Asub = Aeq(:,nu+1:nu+2*nq);
    % x_guess = Ax*z + bx
    bx = Asub\(beq - Aeq(:,1:nu)*u0);
    Ax = -Asub\Aeq(:,nu+2*nq+1:nu+2*nq+nbool);
    assert(any(any(Aeq(:,2*nq+nu+nbool+1:end))) == 0); % making sure we're not missing anything
    
    zguess = pathlcp(M + W*Ax,w+W*bx);
    xguess = Ax*zguess + bx;
    
    if 0
      ind_z_active = [1;1;0;0;1;1;0;0;1;1;0;0;0;0;1;1];
      
%       Aeq = [Aeq;M(ind_z_active == 1,:)*Iz_lc + W(ind_z_active == 1,:)*Ix];
%       beq = [beq;-w(ind_z_active == 1)];
      lb(end-nbool + find(ind_z_active == 1)) = .5;
      ub(end-nbool + find(ind_z_active == 0)) = 0;
      ub(end-nbool-nf+find(ind_z_active == 0)) = 0;
    end
    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    
    %     Hqp = Iu'*h*R*Iu + Ix'*S*Ix;
    %     fqp = -x0'*S*Ix - u0'*h*R*Iu;
    
    % was this!
    %     Hqp = h*Iu'*R*Iu + Ix'*S*Ix;
    %     fqp = -x0'*S*Ix - h*u0'*R*Iu;
%     R = 0;
    %after adding projection
    Hqp = h*Iu'*R*Iu + Ix'*S*Ix;
    fqp = (- x0)'*S*Ix - h*u0'*R*Iu;
    
    % add cost off of the surface
    K = 000*ones(1,length(supp_idx));
    fq_add = K*n(supp_idx,:);
    fqp = fqp + fq_add*Iq;
    
    %cost on boolean
    Kbool = 0*ones(1,length(supp_idx));
    fqp = fqp -Kbool*Ibool(supp_idx,:);

%     if any(n_err)
%     J = [n_err];
% %     K = 10000;
% %     Hqp = Hqp + K*Iq'*(J'*J)*Iq;
% %     fqp = fqp - K*q0'*(J'*J)*Iq;
% 
% 
%       K = 10000;
%       fq_add = K*J;
%       Hq_add = 0;
%       fqp = fqp + fq_add*Iq;
% 
%     else
%       Hq_add = 0;
%       fq_add = zeros(1,nq);
%     end
%     
    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------

    REG = 1e-8;
    obj.gurobi_options.method = 1;
    obj.gurobi_options.outputflag = 1;
    obj.gurobi_options.mipgap = 1e-9;
    obj.gurobi_options.mipfocus = 1;
    obj.gurobi_options.intfeastol = 1e-9;
    

    IR = eye(nparams);  
    lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
    Ain_fqp = full([-IR(lbind,:); IR(ubind,:)]);
    bin_fqp = [-lb(lbind); ub(ubind)];
    
    % then call gurobi
    %       disp('QPController: failed over to gurobi');
    model.Q = sparse(Hqp + REG*eye(nparams));
    model.A = [Aeq;Aineq];
    model.rhs = [beq;bineq];
    model.sense = [obj.eq_array(1:length(beq));obj.ineq_array(1:length(bineq))];
    model.lb = lb;
    model.ub = ub;
    model.vtype = [repmat('C',nparams-nbool,1);repmat('I',nbool,1)];
    
    model.obj = fqp;
    if obj.gurobi_options.method==2
      % see drake/algorithms/QuadraticProgram.m solveWGUROBI
      model.Q = .5*model.Q;
    end
    
    if (any(any(isnan(model.Q))) || any(isnan(model.obj)) || any(any(isnan(model.A))) || any(isnan(model.rhs)) || any(isnan(model.lb)) || any(isnan(model.ub)))
      keyboard;
    end
    
%     result = gurobi(model,obj.gurobi_options);
%     
%     if ~strcmp(result.status,'OPTIMAL')
%       keyboard
%       y = obj.output_nlp(t,[],x);
%       return
%     end
% 
%     alpha = result.x;
%     beta=Ibeta*alpha

% model.Q = Ix'*S*Ix + REG*eye(nparams);
% fqp = -x0'*S*Ix;


    % for mosek
    prob.c = fqp;
    [i,j,Qij] = find(model.Q);
    Qinds = find(i >= j);
    prob.qosubi = i(Qinds);
    prob.qosubj = j(Qinds);
    prob.qoval = Qij(Qinds);
    prob.a = [Aeq;Aineq];
    prob.blc = [beq;-inf(length(bineq),1)];
    prob.buc = [beq;bineq];
    prob.blx = lb;
    prob.bux = ub;
    prob.ints.sub = nparams-nbool+1:nparams;
    prob.sol.int.xx = [u0;xguess;zguess;zguess > 1e-6];
%     [r,res]=mosekopt('param');
%     param = res.param;
%     param.MSK_DPAR_MIO_TOL_ABS_RELAX_INT = 1e-3;
%     param.MSK_DPAR_MIO_TOL_REL_RELAX_INT = 1e-4;
%     param.MSK_DPAR_MIO_TOL_ABS_GAP = 0;
%     param.MSK_DPAR_MIO_TOL_REL_GAP = 1e-2;
%     param.MSK_DPAR_MIO_NEAR_TOL_ABS_GAP = 0;
%     param.MSK_DPAR_MIO_NEAR_TOL_REL_GAP = 1e-1;

if false
    
%     [~,res] = mosekopt('minimize',prob); 
    [info.console,msk_r,res] = evalc('mosekopt(''minimize'', prob);');
    
    alpha = res.sol.int.xx;
    
elseif true
  [x_opt,f_opt] = solveQPCC({Ix*model.Q*Ix'},{Iu*model.Q*Iu'},{prob.c*Ix'},{prob.c*Iu'},{H},{C},{B},{n},D(1),{phi},{n},{0*D{1}*qd},{[D{1}*0 D{1}]},mu,r.umin,r.umax,q,qd,h);
  alpha = [x_opt(2*nq+1:2*nq+nu);x_opt(1:2*nq);x_opt(2*nq+nu+1:end);zeros(16,1)];  
else
  [info.console,msk_r,res] = evalc('mosekopt(''minimize'', prob);');
  
  alpha = res.sol.int.xx;
  [x_opt,f_opt] = solveQPCC({Ix*model.Q*Ix'},{Iu*model.Q*Iu'},{prob.c*Ix'},{prob.c*Iu'},{H},{C},{B},{n},D(1),{phi},{n},{0*D{1}*qd},{[D{1}*0 D{1}]},mu,r.umin,r.umax,q,qd,h);
  
  if f_opt < res.sol.int.pobjval
    alpha = [x_opt(2*nq+1:2*nq+nu);x_opt(1:2*nq);x_opt(2*nq+nu+1:end);zeros(16,1)];
  end
end

    y = Iu*alpha;    
    Ibool*alpha;
    % predicted normal impulse
%     impulse_pred = Dbar*Ibeta*alpha
%     qd_pred = Iqd*alpha
    
    if supp_idx > 1 && false
      [y_prev,cost_prev] = obj.output_prevsup(t,[],x);
      cost = .5*alpha'*Hqp*alpha + fqp*alpha;
      
      if cost_prev < cost
        y = y_prev;
      end
    end
    
      cost = .5*alpha'*Hqp*alpha + fqp*alpha;
      orig_cost = cost;
    
    if false && any(n_err)% | true% & false
      % check other permutations here
      % consider adding phi >= 0 for all other contacts
%       keyboard
      I = find(phi_all < 1e-3);
      for i=1:2^length(I),
        ind_i = I(de2bi(i-1,length(I)) == true);
        [y2,cost2,data]=output_byind(obj,t,[],x,ind_i,Hq_add,fq_add,proj_data);
        if cost2 < cost          
          cost = cost2;
          y = y2;
        end
      end
    end
    
    xpred = Ix*alpha;
    
    if obj.offset_x
      x0t(1) = x0t(1) + obj.controller_data.xoffset;
    end
    
    Vpred = (xpred-x0)'*S*(xpred-x0);
    Vcur = (x-x0t)'*S0t*(x-x0t);
%     x_proj = A_proj*alpha - b_proj;
%     Vproj = (x_proj - x0)'*S*(x_proj - x0);
    display(sprintf('t=%f, V=%.3f, Vp=%.3f, dV=%.3f, dcost=%.3f',t,Vcur,Vpred,Vpred - Vcur, orig_cost - cost));
%     if Vpred > 20
%       keyboard
%     end
%     if any(phi_err)
%       keyboard
%     end

% kinsol_pred = doKinematics(r,Iq*alpha);
% phi_pred = contactConstraints(r,kinsol_pred,false,struct('terrain_only',1));
% [phi_all phi_pred]
Ilambda*alpha;
  end
    
  function y=output(obj,t,~,x)
    y = obj.output_miqp(t,[],x);
    return;

    doProj = false;
    
    
    ctrl_data = obj.controller_data;
      
    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 
            
    supp_idx = find(ctrl_data.support_times<=t,1,'last');

    h=obj.timestep;

%     if ctrl_data.B_is_time_varying
%       if isa(ctrl_data.B,'Trajectory')
%         B_ls = fasteval(ctrl_data.B,t);
%       else
%         B_ls = fasteval(ctrl_data.B{supp_idx},t);
%       end
%     else
%       B_ls = ctrl_data.B; 
%     end
    R = ctrl_data.R;
    if (ctrl_data.lqr_is_time_varying)
      if isa(ctrl_data.S,'Trajectory')
        S = fasteval(ctrl_data.S,t+h);
      else
        S = fasteval(ctrl_data.S{supp_idx},t+h);
      end
      x0 = fasteval(ctrl_data.x0,t+h);
      u0 = fasteval(ctrl_data.u0,t);
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
    end
    q0 = x0(1:nq);

    % get phi for planned contact groups
    % if phi < threshold, then add to active contacts
    % else, add to desired contacts

    kinsol = doKinematics(r,q,true,true,qd);
    rigid_body_support_state = ctrl_data.supports(supp_idx);
    
    planned_supports = rigid_body_support_state.bodies;
    planned_contact_groups = rigid_body_support_state.contact_groups;
    %planned_num_contacts = rigid_body_support_state.num_contact_pts;      

    dim = 2; % 2D or 3D

    Jn = [];
    Jndot = [];
    phi_err = [];
    Dbar = [];
    xp = [];
    Jp = [];
    Jpdot = [];
    nc = 0;
    
    % TODO: do something about contacts that are unplanned--they can
    % definitely help
    
    phi_active = [];
    n_active = zeros(0,nq);
    D_active= zeros(0,nq);
    n_err = zeros(0,nq);
    D_err = zeros(0,nq);
    n_mode = zeros(0,nq);
    D_mode = zeros(0,nq);
    phi_mode = [];
    
    
    
    [phi_all,~,~,~,~,~,~,~,n_all] = contactConstraints(r,kinsol,false,struct('terrain_only',1));
    
    for j=1:length(planned_supports)
      % ridiculously inefficient for testing
      [phi,~,~,~,~,~,~,~,n,D] = contactConstraints(r,kinsol,false,struct('terrain_only',1,...
          'body_idx',[1,planned_supports(j)],'collision_groups',planned_contact_groups(j)));
      [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',1,...
          'body_idx',[1,planned_supports(j)],'collision_groups',planned_contact_groups(j)));
      
      active_ind = phi<=obj.contact_threshold*10;
      phi_err = [phi_err;-phi(~active_ind)];
      
      nc = nc+sum(active_ind);
      Dbar = [Dbar, vertcat(JB{active_ind})']; 
%       ndot = matGradMult(dn,qd);
      Jn = [Jn; n(active_ind,:)];
%       Jndot = [Jndot; ndot(~active_ind,:)];

       % hacky here because we're lacking planar system support
      terrain_pts = getTerrainContactPoints(r,planned_supports(j),planned_contact_groups(j));
      pts = [terrain_pts.pts];
      pts = pts(:,active_ind);

      xz_pts = pts([1 3],:);
      [xp_j,Jp_j] = forwardKin(r,kinsol,planned_supports(j),xz_pts,0);
      Jpdot_j = forwardJacDot(r,kinsol,planned_supports(j),pts,0);
          
      xp = [xp,xp_j];
      Jp = [Jp;Jp_j];
      Jpdot = [Jpdot;Jpdot_j];
      
      
      phi_active = [phi_active;phi(active_ind)];
      n_active = [n_active;n(active_ind,:)];
      D_active = [D_active;D{1}(active_ind,:)];
      n_err = [n_err;n(~active_ind,:)];
      D_err = [D_err;D{1}(~active_ind,:)];
      n_mode = [n_mode;n];
      D_mode = [D_mode;D{1}];
      phi_mode = [phi_mode;phi];
    end
    if exist('xz_pts') && ~isempty(xz_pts)
      % compute foot placement error
      kinsol0 = r.doKinematics(q0);
      xp0 = forwardKin(r,kinsol0,planned_supports(j),xz_pts,0);
      obj.controller_data.xoffset = -1*(mean(xp0(1,:)-xp_j(1,:))); % not quite right, need to take this over all bodies in contact
    end
    
    if dim==2
       % delete y rows
      yind = 2:3:nc*3;
      Jpdot(yind,:) = [];
      Jp = sparse(Jp);
      Jpdot = sparse(Jpdot);

      nd = 2; % for friction cone approx, hard coded for now
    elseif dim==3
      nd = 4; % for friction cone approx, hard coded for now
    end
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    neps = nc*dim;
    if obj.offset_x
      xoffset = obj.controller_data.xoffset;
      if norm(xoffset) > .001
        display(sprintf('offset=%.3f',xoffset));
      end
      x0(1) = x0(1) + obj.controller_data.xoffset;
      q0(1) = q0(1) + obj.controller_data.xoffset;
    end
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    nu = getNumInputs(r);
    nf = nc*nd; % number of contact force variables
    neta = length(phi_err);
    if doProj
      nLambda = size(n_mode,1);
      nLambdaf = size(D_mode,1);
    else
      nLambda = 0;
      nLambdaf = 0;
      n_mode = zeros(0,nq);
      D_mode = zeros(0,nq);
    end
    nparams = nu+2*nq+nf+neps+neta+nLambda+nLambdaf;
    Iu = zeros(nu,nparams); Iu(:,1:nu) = eye(nu);
    Iq = zeros(nq,nparams); Iq(:,nu+(1:nq)) = eye(nq);
    Iqd = zeros(nq,nparams); Iqd(:,nu+nq+(1:nq)) = eye(nq);
    Ix = zeros(2*nq,nparams); Ix(:,nu+(1:2*nq)) = eye(2*nq);
    Ibeta = zeros(nf,nparams); Ibeta(:,nu+2*nq+(1:nf)) = eye(nf);
    Ieps = zeros(neps,nparams); % cpos_ddot slack vars
    Ieps(:,nu+2*nq+nf+(1:neps)) = eye(neps);
    Ieta = zeros(neta,nparams); % phi_ddot slack vars
    Ieta(:,nu+2*nq+nf+neps+(1:neta)) = eye(neta);
    ILambda = zeros(nLambda,nparams); %Lambda impulse slack vars for projection
    ILambda(:,nu+2*nq+nf+neps+neta+(1:nLambda)) = eye(nLambda);
    ILambdaf = zeros(nLambdaf,nparams); %Lambda impulse slack vars for projection
    ILambdaf(:,nu+2*nq+nf+neps+neta+nLambda+(1:nLambdaf)) = eye(nLambdaf);
    
    

            
    
    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [r.umin' obj.jlmin' -inf*ones(1,nq) zeros(1,nf) -obj.cpos_slack_limit*ones(1,neps) -obj.phi_slack_limit*ones(1,neta), zeros(1,nLambda), -inf(1,nLambdaf)]'; % qddot/contact forces/slack vars
    ub = [r.umax' obj.jlmax' inf*ones(1,nq) inf*ones(1,nf) obj.cpos_slack_limit*ones(1,neps) obj.phi_slack_limit*ones(1,neta), inf(1,nLambda+nLambdaf)]';

    Aeq_ = cell(1,4);
    beq_ = cell(1,3);

    % dynamics constraints

    Hinv = inv(H);
    h_Hinv = h*Hinv;
    if nc>0
      Aeq_{1} = Iqd - h_Hinv*B*Iu - Hinv*Dbar*Ibeta;
    else
      Aeq_{1} = Iqd - h_Hinv*B*Iu;
    end
    beq_{1} = qd - h_Hinv*C;

    Aeq_{2} = Iq - h*Iqd;
    beq_{2} = q;%+h*qd;

    if nc > 0
%       Ddot = matGradMult(dD{1},qd);
%       Aeq_{3} = n*Iqd + ndot*Iq;
%       beq_{3} = ndot*q;
%       Aeq_{3} = Aeq_{3}(active_ind,:);
%       beq_{3} = beq_{3}(active_ind,:);

%       Aeq_{4} = D{1}*Iqd + Ddot*Iq;
%       beq_{4} = Ddot*q;
        Aeq_{3} = n_active*Iqd*h;
        beq_{3} = -phi_active;
      Aeq_{4} = D_active*Iqd;
      beq_{4} = zeros(size(Aeq_{4},1),1);
%       
%       Aeq_{4} = Aeq_{4}(active_ind,:);
%       beq_{4} = beq_{4}(active_ind,:);
      
      if ~isempty(phi_err)
%         Aeq_{5} = n_err*Iqd*h;
%         beq_{5} = -phi_err;
      else
%         Aeq_{3} = n*Iqd*h;
%         beq_{3} = -phi;
%         Aeq_{3} = Aeq_{3}(active_ind,:);
%         beq_{3} = beq_{3}(active_ind,:);
      end
      
%       % relative acceleration constraint
%       Aeq_{3} = Jp*Iqdd + Ieps;
%       beq_{3} = -Jpdot*qd - obj.Kp_accel*Jp*qd; 
    end
% 
%     if ~isempty(phi_err)
%       phi_ddot_desired = obj.Kp_phi*phi_err - obj.Kd_phi*(Jndot*qd);
%       Aeq_{3} = Jn*Iqdd + Ieta;
%       beq_{3} = -Jndot*qd + phi_ddot_desired; 
%     end
%     

    
    
    
    % inequality constraints on phi
    Aineq = -n_all*Iqd*h;
    bineq = phi_all;
    
%     phi = contactConstraints(r,x0(1:nq),false,struct('terrain_only',1));
%     active_constraints = ctrl_data.mode_data{supp_idx}.constraint_ind;
%     dynamicsFun = @(t,x,u) constrainedDynamics(r,t,x0,u,active_constraints);
%     [~,dxd] = geval(dynamicsFun,t,x0,u0,struct('grad_method','numerical'));
%     B_ls = dxd(:,getNumStates(r)+1+(1:nu));

    %----------------------------------------------------------------------
    % Projection operators onto manifold ----------------------------------
    % x_proj = A_proj*x + b_proj
    if doProj
    proj_data.Ax = [eye(nq) - pinv(n_mode)*n_mode, zeros(nq);zeros(nq), eye(nq)];
    proj_data.AL = [zeros(nq,nLambda+nLambdaf);Hinv*n_mode', Hinv*D_mode'];
    proj_data.b_proj = [-pinv(n_mode)*(phi_mode + n_mode*q);
              zeros(nq,1)];
    proj_data.nLambda = nLambda;
    proj_data.nLambdaf = nLambdaf;
    proj_data.n_mode = n_mode;
    proj_data.D_mode = D_mode;
    
    else
      proj_data.nLambda = 0;
      proj_data.nLambdaf = 0;
      proj_data.Ax = eye(2*nq);
      proj_data.AL = zeros(2*nq,0);
      proj_data.b_proj = zeros(2*nq,1);
      proj_data.n_mode = zeros(0,nq);
      proj_data.D_mode = zeros(0,nq);
    end
    
    A_proj = proj_data.Ax*Ix + proj_data.AL*[ILambda; ILambdaf];
    b_proj = proj_data.b_proj;    
    
    % additional constraint here
    Aeq_{end+1} = [n_mode;D_mode]*A_proj(nq+1:end,:);
    beq_{end+1} = -[n_mode;D_mode]*b_proj(nq+1:end);
    
%     Aineq = [Aineq;ILambdaf - ILambda;-ILambdaf - ILambda];
%     bineq = [bineq;zeros(2*nLambda,1)];
    
%     %phidot_proj >= 0
%     Aineq = [Aineq;-n_mode*A_proj(nq+1:end,:)];
%     bineq = [bineq;0*n_mode*b_proj(nq+1:end)];
    
%     %phidot_proj >= phidot_pred
%     Aineq = [Aineq;-n_mode*(A_proj(nq+1:end,:) - Iqd)];
%     bineq = [bineq;0*n_mode*b_proj(nq+1:end)];
    
    %psi_proj
%     for i=1:size(D_mode,1),
%       if D_mode(i,:)*qd >= 0
%         %psi_proj >= 0
%         Aineq = [Aineq;-D_mode(i,:)*A_proj(nq+1:end,:)];
%         bineq = [bineq;0];
%         
%         %psi_proj <= psidot_pred
%         Aineq = [Aineq;D_mode(i,:)*(A_proj(nq+1:end,:) - Iqd)];
%         bineq = [bineq;0];
%       else
%         %psi_proj <= 0
%         Aineq = [Aineq;D_mode(i,:)*A_proj(nq+1:end,:)];
%         bineq = [bineq;0];
%         
%         %psi_proj >= psidot_pred
%         Aineq = [Aineq;-D_mode(i,:)*(A_proj(nq+1:end,:) - Iqd)];
%         bineq = [bineq;0];
%       end
%     end

    
    
    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});
    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    
    %     Hqp = Iu'*h*R*Iu + Ix'*S*Ix;
    %     fqp = -x0'*S*Ix - u0'*h*R*Iu;
    
    % was this!
    %     Hqp = h*Iu'*R*Iu + Ix'*S*Ix;
    %     fqp = -x0'*S*Ix - h*u0'*R*Iu;
    
    %after adding projection
    Hqp = h*Iu'*R*Iu + A_proj'*S*A_proj;
    fqp = (-b_proj - x0)'*S*A_proj - h*u0'*R*Iu;
    
    % add cost off of the surface
    if any(n_err)
    J = [n_err];
%     K = 10000;
%     Hqp = Hqp + K*Iq'*(J'*J)*Iq;
%     fqp = fqp - K*q0'*(J'*J)*Iq;


      K = 10000;
      fq_add = K*J;
      Hq_add = 0;
      fqp = fqp + fq_add*Iq;

    else
      Hq_add = 0;
      fq_add = zeros(1,nq);
    end

%     if nc > 0
%       Hqp(nu+2*nq+(1:nf),nu+2*nq+(1:nf)) = obj.w_grf*eye(nf); 
%       Hqp(nu+2*nq+nf+(1:neps),nu+2*nq+nf+(1:neps)) = obj.w_cpos_slack*eye(neps); 
%       Hqp(nu+2*nq+nf+neps+(1:neta),nu+2*nq+nf+neps+(1:neta)) = obj.w_phi_slack*eye(neta); 
%     end
%     
    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------

    REG = 1e-8;

    IR = eye(nparams);  
    lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
    Ain_fqp = full([-IR(lbind,:); IR(ubind,:)]);
    bin_fqp = [-lb(lbind); ub(ubind)];

    % call fastQPmex first
    QblkDiag = {Hqp(1:(nu+2*nq),1:(nu+2*nq)) + REG*eye(nu+2*nq), ...
                obj.w_grf*ones(nf,1) + REG*ones(nf,1), ...
                obj.w_cpos_slack*ones(neps,1) + REG*ones(neps,1), ...
                obj.w_phi_slack*ones(neta,1) + REG*ones(neta,1)};
              
    Aeq_fqp = full(Aeq);
    % NOTE: model.obj is 2* f for fastQP!!!
%     [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

    if true || info_fqp<0
      % then call gurobi
%       disp('QPController: failed over to gurobi');
      model.Q = sparse(Hqp + REG*eye(nparams));
      model.A = [Aeq;Aineq];
      model.rhs = [beq;bineq];
      model.sense = [obj.eq_array(1:length(beq));obj.ineq_array(1:length(bineq))];
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
      
      if ~strcmp(result.status,'OPTIMAL')
        keyboard
        y = obj.output_nlp(t,[],x);
        return
      end

      qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
      obj.controller_data.qp_active_set = qp_active_set;
    end
%     beta=Ibeta*alpha
    y = Iu*alpha;    
    
    % predicted normal impulse
%     impulse_pred = Dbar*Ibeta*alpha
%     qd_pred = Iqd*alpha
    
    if supp_idx > 1 && false
      [y_prev,cost_prev] = obj.output_prevsup(t,[],x);
      cost = .5*alpha'*Hqp*alpha + fqp*alpha;
      
      if cost_prev < cost
        y = y_prev;
      end
    end
    
      cost = .5*alpha'*Hqp*alpha + fqp*alpha;
      orig_cost = cost;
    
    if any(n_err) & false% | true% & false
      % check other permutations here
      % consider adding phi >= 0 for all other contacts
%       keyboard
      I = find(phi_all < 1e-3);
      for i=1:2^length(I),
        ind_i = I(de2bi(i-1,length(I)) == true);
        [y2,cost2,data]=output_byind(obj,t,[],x,ind_i,Hq_add,fq_add,proj_data);
        if cost2 < cost          
          cost = cost2;
          y = y2;
        end
      end
    end
    
    xpred = Ix*alpha;
    x0t = fasteval(ctrl_data.x0,t);
    if obj.offset_x
      x0t(1) = x0t(1) + obj.controller_data.xoffset;
    end
    S0t = ctrl_data.S{supp_idx}.eval(t);
    Vpred = (xpred-x0)'*S*(xpred-x0);
    Vcur = (x-x0t)'*S0t*(x-x0t);
    x_proj = A_proj*alpha - b_proj;
    Vproj = (x_proj - x0)'*S*(x_proj - x0);
    display(sprintf('t=%f, V=%.3f, Vp=%.3f, dV=%.3f, dcost=%.3f',t,Vcur,Vpred,Vpred - Vcur, orig_cost - cost));
%     if Vpred > 20
%       keyboard
%     end
%     if any(phi_err)
%       keyboard
%     end

% kinsol_pred = doKinematics(r,Iq*alpha);
% phi_pred = contactConstraints(r,kinsol_pred,false,struct('terrain_only',1));
% [phi_all phi_pred]
ILambda*alpha;
  end
  
  function [y,cost,data]=output_byind(obj,t,~,x,ind,Hq_add,fq_add,proj_data)
    ctrl_data = obj.controller_data;
    
    if nargin < 6
      Hq_add = 0;
    end
    
    if nargin < 7
      fq_add = 0;
    end    

    r = obj.robot;
    nq = obj.numq; 
    q = x(1:nq); 
    qd = x(nq+(1:nq)); 
            
    supp_idx = find(ctrl_data.support_times<=t,1,'last');

    h=obj.timestep;

%     if ctrl_data.B_is_time_varying
%       if isa(ctrl_data.B,'Trajectory')
%         B_ls = fasteval(ctrl_data.B,t);
%       else
%         B_ls = fasteval(ctrl_data.B{supp_idx},t);
%       end
%     else
%       B_ls = ctrl_data.B; 
%     end
    R = ctrl_data.R;
    if (ctrl_data.lqr_is_time_varying)
      if isa(ctrl_data.S,'Trajectory')
        S = fasteval(ctrl_data.S,t+h);
      else
        S = fasteval(ctrl_data.S{supp_idx},t+h);
      end
      x0 = fasteval(ctrl_data.x0,t+h);
      u0 = fasteval(ctrl_data.u0,t);
    else
      S = ctrl_data.S;
      x0 = ctrl_data.x0;
      u0 = ctrl_data.u0;
    end
    q0 = x0(1:nq);

    % get phi for planned contact groups
    % if phi < threshold, then add to active contacts
    % else, add to desired contacts

    kinsol = doKinematics(r,q,true,true,qd);
    %planned_num_contacts = rigid_body_support_state.num_contact_pts;      

    dim = 2; % 2D or 3D
    
    [phi_all,~,~,~,~,~,~,~,n_all,D] = contactConstraints(r,kinsol,false,struct('terrain_only',1));
    [~,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',1));
    
    nc = length(ind);
    Dbar = vertcat(JB{ind})';
    n_active = n_all(ind,:);
    phi_active = phi_all(ind);
    D_active = D{1}(ind,:);
  
%        % hacky here because we're lacking planar system support
%       terrain_pts = getTerrainContactPoints(r,planned_supports(j),planned_contact_groups(j));
%       pts = [terrain_pts.pts];
%       pts = pts(:,active_ind);

%       xz_pts = pts([1 3],:);
%       [xp_j,Jp_j] = forwardKin(r,kinsol,planned_supports(j),xz_pts,0);
%       Jpdot_j = forwardJacDot(r,kinsol,planned_supports(j),pts,0);
          
%       xp = [xp,xp_j];
%       Jp = [Jp;Jp_j];
%       Jpdot = [Jpdot;Jpdot_j];
      
      
%       phi_active = [phi_active;phi(active_ind)];
%       n_active = [n_active;n(active_ind,:)];
%       D_active = [D_active;D{1}(active_ind,:)];
%       n_err = [n_err;n(~active_ind,:)];
%       D_err = [D_err;D{1}(~active_ind,:)];
    
      
%     if exist('xz_pts') && ~isempty(xz_pts)
%       % compute foot placement error
%       kinsol0 = r.doKinematics(q0);
%       xp0 = forwardKin(r,kinsol0,planned_supports(j),xz_pts,0);
%       obj.controller_data.xoffset = -1*(mean(xp0(1,:)-xp_j(1,:))); % not quite right, need to take this over all bodies in contact
%     end
    
    if dim==2
       % delete y rows
%       yind = 2:3:nc*3;
%       Jpdot(yind,:) = [];
%       Jp = sparse(Jp);
%       Jpdot = sparse(Jpdot);

      nd = 2; % for friction cone approx, hard coded for now
    elseif dim==3
      nd = 4; % for friction cone approx, hard coded for now
    end
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    neps = nc*dim;
    if obj.offset_x
      xoffset = obj.controller_data.xoffset;
%       display(sprintf('offset=%.3f',xoffset));
      x0(1) = x0(1) + obj.controller_data.xoffset;
    end
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    nu = getNumInputs(r);
    nf = nc*nd; % number of contact force variables
    neta = 0;%length(phi_err);
    nLambda = proj_data.nLambda;
    nLambdaf = proj_data.nLambdaf;
    nparams = nu+2*nq+nf+neps+neta+nLambda+nLambdaf;
    Iu = zeros(nu,nparams); Iu(:,1:nu) = eye(nu);
    Iq = zeros(nq,nparams); Iq(:,nu+(1:nq)) = eye(nq);
    Iqd = zeros(nq,nparams); Iqd(:,nu+nq+(1:nq)) = eye(nq);
    Ix = zeros(2*nq,nparams); Ix(:,nu+(1:2*nq)) = eye(2*nq);
    Ibeta = zeros(nf,nparams); Ibeta(:,nu+2*nq+(1:nf)) = eye(nf);
    Ieps = zeros(neps,nparams); % cpos_ddot slack vars
    Ieps(:,nu+2*nq+nf+(1:neps)) = eye(neps);
    Ieta = zeros(neta,nparams); % phi_ddot slack vars
    Ieta(:,nu+2*nq+nf+neps+(1:neta)) = eye(neta);
    ILambda = zeros(nLambda,nparams); %Lambda impulse slack vars for projection
    ILambda(:,nu+2*nq+nf+neps+neta+(1:nLambda)) = eye(nLambda);
    ILambdaf = zeros(nLambdaf,nparams); %Lambda impulse slack vars for projection
    ILambdaf(:,nu+2*nq+nf+neps+neta+nLambda+(1:nLambdaf)) = eye(nLambdaf);
    
    
    
    A_proj = proj_data.Ax*Ix + proj_data.AL*[ILambda; ILambdaf];
    b_proj = proj_data.b_proj;
    
    
    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------
    lb = [r.umin' obj.jlmin' -inf*ones(1,nq) zeros(1,nf) -obj.cpos_slack_limit*ones(1,neps) -obj.phi_slack_limit*ones(1,neta), zeros(1,nLambda), -inf(1,nLambdaf)]'; % qddot/contact forces/slack vars
    ub = [r.umax' obj.jlmax' inf*ones(1,nq) inf*ones(1,nf) obj.cpos_slack_limit*ones(1,neps) obj.phi_slack_limit*ones(1,neta), inf(1,nLambda+nLambdaf)]';


    Aeq_ = cell(1,4);
    beq_ = cell(1,3);

    % dynamics constraints

    Hinv = inv(H);
    h_Hinv = h*Hinv;
    if nc>0
      Aeq_{1} = Iqd - h_Hinv*B*Iu - Hinv*Dbar*Ibeta;
    else
      Aeq_{1} = Iqd - h_Hinv*B*Iu;
    end
    beq_{1} = qd - h_Hinv*C;

    Aeq_{2} = Iq - h*Iqd;
    beq_{2} = q;%+h*qd;

    if nc > 0
%       Ddot = matGradMult(dD{1},qd);
%       Aeq_{3} = n*Iqd + ndot*Iq;
%       beq_{3} = ndot*q;
%       Aeq_{3} = Aeq_{3}(active_ind,:);
%       beq_{3} = beq_{3}(active_ind,:);

%       Aeq_{4} = D{1}*Iqd + Ddot*Iq;
%       beq_{4} = Ddot*q;
        Aeq_{3} = n_active*Iqd*h;
        beq_{3} = -phi_active;
      Aeq_{4} = D_active*Iqd;
      beq_{4} = zeros(size(Aeq_{4},1),1);
%       
%       Aeq_{4} = Aeq_{4}(active_ind,:);
%       beq_{4} = beq_{4}(active_ind,:);
      
    end
    
    % additional constraint here
    n_mode = proj_data.n_mode;
    D_mode = proj_data.D_mode;
%     Aeq_{end+1} = [n_mode;D_mode]*A_proj(nq+1:end,:);
%     beq_{end+1} = -[n_mode;D_mode]*b_proj(nq+1:end);

    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});    
    
    % inequality constraints on phi
    Aineq = -n_all*Iqd*h;
    bineq = phi_all;
           
    Aineq = [Aineq;ILambdaf - ILambda;-ILambdaf - ILambda];
    bineq = [bineq;zeros(2*nLambda,1)];
    
    %phidot_proj >= 0
    Aineq = [Aineq;-n_mode*A_proj(nq+1:end,:)];
    bineq = [bineq;0*n_mode*b_proj(nq+1:end)];
    
    %phidot_proj >= phidot_pred
    Aineq = [Aineq;-n_mode*(A_proj(nq+1:end,:) - Iqd)];
    bineq = [bineq;0*n_mode*b_proj(nq+1:end)];
    
%     %psi_proj
%     for i=1:size(D_mode,1),
%       if D_mode(i,:)*qd >= 0
%         %psi_proj >= 0
%         Aineq = [Aineq;-D_mode(i,:)*A_proj(nq+1:end,:)];
%         bineq = [bineq;0];
%         
%         %psi_proj <= psidot_pred
%         Aineq = [Aineq;D_mode(i,:)*(A_proj(nq+1:end,:) - Iqd)];
%         bineq = [bineq;0];
%       else
%         %psi_proj <= 0
%         Aineq = [Aineq;D_mode(i,:)*A_proj(nq+1:end,:)];
%         bineq = [bineq;0];
%         
%         %psi_proj >= psidot_pred
%         Aineq = [Aineq;-D_mode(i,:)*(A_proj(nq+1:end,:) - Iqd)];
%         bineq = [bineq;0];
%       end
%     end
    
%     phi = contactConstraints(r,x0(1:nq),false,struct('terrain_only',1));
%     active_constraints = ctrl_data.mode_data{supp_idx}.constraint_ind;
%     dynamicsFun = @(t,x,u) constrainedDynamics(r,t,x0,u,active_constraints);
%     [~,dxd] = geval(dynamicsFun,t,x0,u0,struct('grad_method','numerical'));
%     B_ls = dxd(:,getNumStates(r)+1+(1:nu));
    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    
%     Hqp = Iu'*h*R*Iu + Ix'*S*Ix;
%     fqp = -x0'*S*Ix - u0'*h*R*Iu;

    Hqp = h*Iu'*R*Iu + A_proj'*S*A_proj + Iq'*Hq_add*Iq;
    fqp = (-b_proj-x0)'*S*A_proj - h*u0'*R*Iu + fq_add*Iq;    
    
    
    % add cost off of the surface
%     if any(n_err)
%     J = [n_err];
%     K = 0;
%     Hqp = Hqp + K*Iq'*(J'*J)*Iq;
%     fqp = fqp - K*q0'*(J'*J)*Iq;
%     end

    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------

    REG = 1e-8;

    IR = eye(nparams);  
    lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
    Ain_fqp = full([-IR(lbind,:); IR(ubind,:)]);
    bin_fqp = [-lb(lbind); ub(ubind)];

    % call fastQPmex first
    QblkDiag = {Hqp(1:(nu+2*nq),1:(nu+2*nq)) + REG*eye(nu+2*nq), ...
                obj.w_grf*ones(nf,1) + REG*ones(nf,1), ...
                obj.w_cpos_slack*ones(neps,1) + REG*ones(neps,1), ...
                obj.w_phi_slack*ones(neta,1) + REG*ones(neta,1)};
              
    Aeq_fqp = full(Aeq);
    % NOTE: model.obj is 2* f for fastQP!!!
%     [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq,ctrl_data.qp_active_set);

    if true || info_fqp<0
      % then call gurobi
%       disp('QPController: failed over to gurobi');
      model.Q = sparse(Hqp + REG*eye(nparams));
      model.A = [Aeq;Aineq];
      model.rhs = [beq;bineq];
      model.sense = [obj.eq_array(1:length(beq));obj.ineq_array(1:length(bineq))];
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
      
      if ~strcmp(result.status,'OPTIMAL')
        keyboard
      end

      qp_active_set = find(abs(Ain_fqp*alpha - bin_fqp)<1e-6);
      obj.controller_data.qp_active_set = qp_active_set;
    end
%     beta=Ibeta*alpha
    y = Iu*alpha;    
    
    % predicted normal impulse
%     impulse_pred = Dbar*Ibeta*alpha
%     qd_pred = Iqd*alpha
%     
%     xpred = Ix*alpha;
%     x0t = fasteval(ctrl_data.x0,t);
%     S0t = ctrl_data.S{supp_idx}.eval(t);
%     Vpred = (xpred-x0)'*S*(xpred-x0);
%     display(sprintf('t=%f, V=%f, Vp=%f',t,(x-x0t)'*S0t*(x-x0t),Vpred));

    cost = .5*alpha'*Hqp*alpha + fqp*alpha;
    data = struct();
  end  
  end

  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    controller_data; % shared data handle that holds S, h, foot trajectories, etc.
    w_grf; % scalar ground reaction force weight
    w_cpos_slack; % scalar slack var weight
    w_phi_slack; % scalar slack var weight
    cpos_slack_limit; 
    phi_slack_limit; 
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
    timestep
  end
end
