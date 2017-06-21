classdef CassieIDControl < DrakeSystem
  methods
  function obj = CassieIDControl(r,com_des,q_des,options)
    % @param r rigid body manipulator instance
    % @param options structure for specifying objective weights, slack
    % bounds, etc.
    
    if nargin>3
      typecheck(options,'struct');
    else
      options = struct();
    end

%     qddframe = CassieGeneralizedCoordinates(); % input frame for desired qddot
%     input_frame = MultiCoordinateFrame({r.getStateFrame,qddframe});
    input_frame = r.getStateFrame();
    output_frame = r.getInputFrame();
    
    obj = obj@DrakeSystem(0,0,r.getNumStates,r.getNumInputs,true,true);

%     obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.numq = getNumPositions(r);
    obj.numv = getNumVelocities(r);
    
    if isfield(options,'dt')
      % controller update rate
      typecheck(options.dt,'double');
      sizecheck(options.dt,[1 1]);
      dt = options.dt;
    else
      dt = 0.0005;
    end
    obj = setSampleTime(obj,[dt;0]); % sets controller update rate

    if isfield(options,'use_bullet')
      obj.use_bullet = options.use_bullet;
    else
      obj.use_bullet = false;
    end

    % weight for the desired qddot objective term
    if isfield(options,'w_qdd')
      typecheck(options.w_qdd,'double');
      sizecheck(options.w_qdd,[obj.numq 1]); % assume diagonal cost
      obj.w_qdd = options.w_qdd;
    else
      obj.w_qdd = 0.001*ones(obj.numq,1);
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
      obj.slack_limit = 0.1;
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

    obj.using_flat_terrain = true;
    [obj.jlmin, obj.jlmax] = getJointLimits(r);
    
    obj.com_des = com_des;
    obj.q_des = q_des;
  end

%   function y=mimoOutput(obj,~,~,varargin)
  function y=output(obj,~,~,x)
%     x = varargin{1};
%     qddot_des = varargin{2};

    r = obj.robot;
    nq = obj.numq;
    nu = r.getNumInputs;
    q = x(1:nq);
    qd = x(nq+(1:nq));
    
    kinsol = doKinematics(r,q,false,true,qd);

    dim = 3; % 3D
    nd = 4; % for friction cone approx, hard coded for now

    [H,C,B] = manipulatorDynamics(r,q,qd);

    [xcom,Jcom] = getCOM(r,kinsol);
    Jcomdot_times_v = centerOfMassJacobianDotTimesV(r, kinsol);
    
    nc = 4;
    [phi,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet));
    Dbar = vertcat(JB{:})';
    
    terrain_pts = getTerrainContactPoints(r);
    [~,Jp] = terrainContactPositions(r,kinsol,terrain_pts,true);
    Jpdot_times_v = terrainContactJacobianDotTimesV(r, kinsol, terrain_pts);
    Jp = sparse(Jp);
   
    THIGH_LEFT_ID = r.findLinkId('thigh_left');
    THIGH_RIGHT_ID = r.findLinkId('thigh_right');
    HEEL_SPRING_LEFT_ID = r.findLinkId('heel_spring_left');
    HEEL_SPRING_RIGHT_ID = r.findLinkId('heel_spring_right');
    
    [pl1,J_l1] = forwardKin(r,kinsol,THIGH_LEFT_ID,[0;0;0.0045]);
    [pl2,J_l2] = forwardKin(r,kinsol,HEEL_SPRING_LEFT_ID,[0.11877; -0.0001; 0]);
    [pr1,J_r1] = forwardKin(r,kinsol,THIGH_RIGHT_ID,[0;0;0.0045]);
    [pr2,J_r2] = forwardKin(r,kinsol,HEEL_SPRING_RIGHT_ID,[0.11877; -0.0001; 0]);

    Jl = sparse(J_l1-J_l2);
    Jr = sparse(J_r1-J_r2);
%     Jconst = [Jl;Jr];

%     Jl1dot_times_v = forwardJacDotTimesV(r,kinsol,THIGH_LEFT_ID,[0;0;0.0045]);
%     Jl2dot_times_v = forwardJacDotTimesV(r,kinsol,HEEL_SPRING_LEFT_ID,[0.11877; -0.0001; 0]);
%     Jr1dot_times_v = forwardJacDotTimesV(r,kinsol,THIGH_RIGHT_ID,[0;0;0.0045]);
%     Jr2dot_times_v = forwardJacDotTimesV(r,kinsol,HEEL_SPRING_RIGHT_ID,[0.11877; -0.0001; 0]);
%     
%     Jldot_times_v = Jl1dot_times_v-Jl2dot_times_v;
%     Jrdot_times_v = Jr1dot_times_v-Jr2dot_times_v;
    
    
    norm(pl1-pl2)
    norm(pr1-pr2)

    dfl_dq = (1./norm(pl1-pl2)) * ((pl1-pl2)'*Jl);
    dfr_dq = (1./norm(pr1-pr2)) * ((pr1-pr2)'*Jr);

    J4bar = [dfl_dq;dfr_dq];
    
%     Jconst_dot_times_v = [Jldot_times_v;Jrdot_times_v];

    neps = nc*dim;

    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    nf = nc*nd; % number of contact force variables
    ncf = 2; % number of loop constraint forces
    nparams = nq+nu+nf+ncf+neps;
    Iqdd = zeros(nq,nparams); Iqdd(:,1:nq) = eye(nq);
    Iu = zeros(nu,nparams); Iu(:,nq+(1:nu)) = eye(nu);
    Ibeta = zeros(nf,nparams); Ibeta(:,nq+nu+(1:nf)) = eye(nf);
    Iconst = zeros(ncf,nparams); Iconst(:,nq+nu+nf+(1:ncf)) = eye(ncf);
    Ieps = zeros(neps,nparams); Ieps(:,nq+nu+nf+ncf+(1:neps)) = eye(neps);

    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [-inf(1,nq) r.umin'  zeros(1,nf) -inf(1,ncf)  -obj.slack_limit*ones(1,neps)]'; % qddot/u/contact+constraint forces/slack vars
    ub = [inf(1,nq) r.umax' inf(1,nf) inf(1,ncf) obj.slack_limit*ones(1,neps)]';

    Aeq_ = cell(1,4);
    beq_ = cell(1,4);

    % constrained dynamics
    if nc>0
      Aeq_{1} = H*Iqdd - B*Iu - Dbar*Ibeta - J4bar'*Iconst;
    else
      Aeq_{1} = H*Iqdd - B*Iu - J4bar'*Iconst;
    end
    beq_{1} = -C;

    if nc > 0
      % relative acceleration constraint for contacts
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdot_times_v - obj.Kp_accel*Jp*qd;
    end
    % relative acceleration constraint for 4bar
    Aeq_{3} = 0.0005*J4bar*Iqdd;
    beq_{3} = -J4bar * qd;

    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});

%     % linear inequality constraints: Ain*alpha <= bin
%     Ain = sparse(vertcat(Ain_{:}));
%     bin = vertcat(bin_{:});
%     Ain = Ain(bin~=inf,:);
%     bin = bin(bin~=inf);
    Ain = [];
    bin = [];

    Kp_com = 100;
    Kd_com = 1.5*sqrt(Kp_com);
    Kp_qdd = 100*eye(nq);
    Kd_qdd = 1.5*sqrt(Kp_qdd);

    idx = 4+[r.findJointId('knee_shin_passive_left');
          r.findJointId('knee_shin_passive_right');
          r.findJointId('heel_spring_joint_left');
          r.findJointId('heel_spring_joint_right')];
    
    Kp_qdd(idx,idx) = 0;
    Kd_qdd(idx,idx) = 0;
    com_ddot_des = Kp_com*(obj.com_des - xcom) - Kd_com*Jcom*qd;
    Q = 10*eye(3);

    qddot_des = Kp_qdd*(obj.q_des - q) - Kd_qdd*qd;

    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    %
    % min: ybar*Qy*ybar + ubar*R*ubar + (2*S*xbar + s1)*(A*x + B*u) +
    % w_qdd*quad(qddot_ref - qdd) + w_eps*quad(epsilon) +
    % w_grf*quad(beta) + quad(kdot_des - (A*qdd + Adot*qd))
    if nc > 0
      Hqp = Iqdd'*Jcom'*Q*Jcom*Iqdd;
      Hqp(1:nq,1:nq) = Hqp(1:nq,1:nq) + diag(obj.w_qdd);
      Hqp(nq+(1:nu),nq+(1:nu)) = Hqp(nq+(1:nu),nq+(1:nu)) + 1e-6*eye(nu);

      fqp = Jcomdot_times_v'*Q*Jcom*Iqdd;
      fqp = fqp - com_ddot_des'*Q*Jcom*Iqdd;
      fqp = fqp - (obj.w_qdd.*qddot_des)'*Iqdd;

      Hqp(nparams-neps+1:end,nparams-neps+1:end) = obj.w_slack*eye(neps);
    else
      Hqp = Iqdd'*Iqdd;
      fqp = -qddot_des'*Iqdd;
    end

    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------

    REG = 1e-8;

    IR = eye(nparams);
    lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
    Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
    bin_fqp = [bin; -lb(lbind); ub(ubind)];

%     % call fastQPmex first
%     QblkDiag = {Hqp(1:nq,1:nq) + REG*eye(nq), ...
%                 REG*ones(nu,1), ...
%                 REG*ones(nf,1), ...
%                 obj.w_slack*ones(neps,1) + REG*ones(neps,1)};
%     Aeq_fqp = full(Aeq);
%     % NOTE: model.obj is 2* f for fastQP!!!
%     [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq);
% 
    if 1% info_fqp<0
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
    
    cf = Iconst*alpha;
    y = alpha(nq+(1:nu));
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
    com_des;
    q_des;
  end
end
