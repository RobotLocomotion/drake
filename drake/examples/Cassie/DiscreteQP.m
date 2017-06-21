classdef DiscreteQP < DrakeSystem
  methods
  function obj = DiscreteQP(r,com_ref,x_ref,options)
    % @param r rigid body manipulator instance
    % @param options structure for specifying objective weights, slack
    % bounds, etc.
    
    if nargin>3
      typecheck(options,'struct');
    else
      options = struct();
    end

    input_frame = r.getStateFrame();
    output_frame = r.getInputFrame();
    
    obj = obj@DrakeSystem(0,0,r.getNumStates,r.getNumInputs,true,true);

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
    obj.h = dt;
    
    if isfield(options,'use_bullet')
      obj.use_bullet = options.use_bullet;
    else
      obj.use_bullet = false;
    end

    % weight for the desired state objective term
    if isfield(options,'Q_x')
      typecheck(options.Q_x,'double');
      sizecheck(options.Q_x,[obj.numq+obj.numv 1]); % assume diagonal cost
      obj.Q_x = options.Q_x;
    else
      obj.Q_x = [500*ones(obj.numq,1);10*ones(obj.numv,1)];
    end

    % weight for the desired COM state objective term
    if isfield(options,'Q_com')
      typecheck(options.Q_com,'double');
      sizecheck(options.Q_com,[6 1]); % assume diagonal cost
      obj.Q_com = options.Q_com;
    else
      obj.Q_com = 1000*ones(6,1);
    end

    % cost on u
    if isfield(options,'R')
      typecheck(options.R,'double');
      obj.R = options.R;
    else
      obj.R = 0*ones(r.getNumInputs,1);
    end
    
    % weight for slack vars
    if isfield(options,'w_slack')
      typecheck(options.w_slack,'double');
      sizecheck(options.w_slack,1);
      obj.w_slack = options.w_slack;
    else
      obj.w_slack = 0.0;
    end

    % hard bound on slack variable values
    if isfield(options,'slack_limit')
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    else
      obj.slack_limit = 0;
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
    
    obj.com_ref = com_ref;
    obj.x_ref = x_ref;
  end

  function y=output(obj,~,~,x)

    r = obj.robot;
    nq = obj.numq;
    nv = obj.numv;
    nu = r.getNumInputs;
    nx = nq+nv;
    q = x(1:nq);
    qd = x(nq+(1:nq));
    
    kinsol = doKinematics(r,q,false,true,qd);

    dim = 3; % 3D
    nd = 4; % for friction cone approx, hard coded for now

    [H,C,B] = manipulatorDynamics(r,q,qd);

    [com,Jcom] = getCOM(r,kinsol);
    
    nc = 4;
    [phi,~,JB] = contactConstraintsBV(r,kinsol,false,struct('terrain_only',~obj.use_bullet));
    Dbar = vertcat(JB{:})';
    
    terrain_pts = getTerrainContactPoints(r);
    [~,Jp] = terrainContactPositions(r,kinsol,terrain_pts,true);
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

    dfl_dq = (1./norm(pl1-pl2)) * ((pl1-pl2)'*Jl);
    dfr_dq = (1./norm(pr1-pr2)) * ((pr1-pr2)'*Jr);

    J4bar = [dfl_dq;dfr_dq];
    
    
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------

    neps = nc*dim;
    nf = nc*nd; % number of contact force variables
    ncf = 2; % number of loop constraint forces
    nparams = nq+nv+nu+nf+ncf+neps;
    Iq = zeros(nq,nparams); Iq(:,1:nq) = eye(nq);
    Iqd = zeros(nv,nparams); Iqd(:,nq+(1:nv)) = eye(nv);
    Ix = zeros(nx,nparams); Ix(:,1:nx) = eye(nx);
    Iu = zeros(nu,nparams); Iu(:,nq+nv+(1:nu)) = eye(nu);
    Ibeta = zeros(nf,nparams); Ibeta(:,nq+nv+nu+(1:nf)) = eye(nf);
    Iconst = zeros(ncf,nparams); Iconst(:,nq+nv+nu+nf+(1:ncf)) = eye(ncf);
    Ieps = zeros(neps,nparams); Ieps(:,nq+nv+nu+nf+ncf+(1:neps)) = eye(neps);

    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [obj.jlmin' -inf(1,nv) r.umin'  zeros(1,nf) -inf(1,ncf)  -obj.slack_limit*ones(1,neps)]'; % q/qd/u/contact+constraint forces/slack vars
    ub = [obj.jlmax' inf(1,nv) r.umax' inf(1,nf) inf(1,ncf) obj.slack_limit*ones(1,neps)]';

    Aeq_ = cell(1,4);
    beq_ = cell(1,4);

    % constrained dynamics
    if nc>0
      Aeq_{1} = H*Iqd - obj.h*B*Iu - obj.h*Dbar*Ibeta - obj.h*J4bar'*Iconst;
    else
      Aeq_{1} = H*Iqd - obj.h*B*Iu - obj.h*J4bar'*Iconst;
    end
    beq_{1} = -obj.h*C + H*qd;

    if nc > 0
%       relative velocity constraint for contacts
      Aeq_{2} = Jp*Iqd + Ieps;
      beq_{2} = zeros(neps,1);
    end
    % relative acceleration constraint for 4bar
    Aeq_{3} = J4bar*Iqd;
    beq_{3} = zeros(2,1);
    % euler integration constraint
    Aeq_{4} = Iq - obj.h*Iqd;
    beq_{4} = q;
    
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
    
    dc_next =  [obj.h*Jcom; Jcom] * Iqd;
    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
 
    if nc > 0
      % state cost
      Hqp = Ix'*diag(obj.Q_x)*Ix;

      % COM state cost
      Hqp = Hqp + dc_next'*diag(obj.Q_com)*dc_next;

      % input cost
      Hqp(nq+nv+(1:nu),nq+nv+(1:nu)) = Hqp(nq+nv+(1:nu),nq+nv+(1:nu)) + diag(obj.R);

      % slack cost
      Hqp(nparams-neps+1:end,nparams-neps+1:end) = obj.w_slack*eye(neps);

      % linear terms
      fqp = -(obj.Q_x.*obj.x_ref)'*Ix;
      fqp = fqp - (obj.Q_com.*(obj.com_ref +[com;zeros(3,1)]))'*dc_next;
      
    else
        error('uh oh')
    end

    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------

    REG = 1e-8;

    IR = eye(nparams);
    lbind = lb>-999;  ubind = ub<999;  % 1e3 was used like inf above... right?
    Ain_fqp = full([Ain; -IR(lbind,:); IR(ubind,:)]);
    bin_fqp = [bin; -lb(lbind); ub(ubind)];

    % call fastQPmex first
    QblkDiag = {Hqp(1:nx,1:nx) + REG*eye(nx), ...
                obj.R, ...
                REG*ones(nf+ncf,1), ...
                obj.w_slack*ones(neps,1) + REG*ones(neps,1)};
    Aeq_fqp = full(Aeq);
    % NOTE: model.obj is 2* f for fastQP!!!
%     [alpha,info_fqp] = fastQPmex(QblkDiag,fqp,Ain_fqp,bin_fqp,Aeq_fqp,beq);

    if  1%info_fqp<0
        
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
      result.status
    end
    
%     qdn = qd + obj.h*inv(H)*(B*Iu*alpha + Dbar*Ibeta*alpha - C);
%     [qdn,Iqd*alpha]
    
    y = Iu*alpha;
    end
  end
  
  properties (SetAccess=private)
    robot; % to be controlled
    numq;
    numv;
    Q_x; 
    Q_com;
    R;
    h;
    w_slack; % scalar slack var weight
    slack_limit; % maximum absolute magnitude of acceleration slack variable values
    gurobi_options = struct();
    solver=0;
    eq_array = repmat('=',100,1); % so we can avoid using repmat in the loop
    ineq_array = repmat('<',100,1); % so we can avoid using repmat in the loop
    use_bullet;
    using_flat_terrain; % true if using DRCFlatTerrain
    jlmin;
    jlmax;
    com_ref;
    x_ref;
  end
end
