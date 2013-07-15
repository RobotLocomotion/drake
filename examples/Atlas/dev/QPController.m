classdef QPController < MIMODrakeSystem
  % implementation assumes 3D atlas model
  methods
  function obj = QPController(r,zmpdata,options)
    % @param r atlas instance
    % @param options structure for specifying objective weight (w), slack
    % variable limits (slack_limit), and action cost (R)
    typecheck(r,'Atlas');
    typecheck(zmpdata,'SharedDataHandle');
    if nargin>2
      typecheck(options,'struct');
    else
      options = struct();
    end
    
    qddframe = AtlasCoordinates(r);
    supportframe = AtlasBody(r);

    input_frame = MultiCoordinateFrame({qddframe,supportframe,r.getStateFrame});
    output_frame = r.getInputFrame();
    obj = obj@MIMODrakeSystem(0,0,input_frame,output_frame,true,true);
    obj = setSampleTime(obj,[.005;0]); % sets controller update rate
    obj = setInputFrame(obj,input_frame);
    obj = setOutputFrame(obj,output_frame);

    obj.robot = r;
    obj.zmpdata = zmpdata;

    if (isfield(options,'w'))
      typecheck(options.w,'double');
      sizecheck(options.w,1);
      obj.w = options.w;
    end
    
    if (isfield(options,'slack_limit'))
      typecheck(options.slack_limit,'double');
      sizecheck(options.slack_limit,1);
      obj.slack_limit = options.slack_limit;
    end
    
    if (isfield(options,'exclude_torso'))
      typecheck(options.exclude_torso,'logical');
    else
      options.exclude_torso = false;
    end
   
    if options.exclude_torso
      % perform unconstrained minimization to compute accelerations for a 
      % subset of atlas DOF, then solve for inputs (then threshold).
      % generally these should be the joints for which the columns of the 
      % contact jacobian are zero. The remaining dofs are indexed in con_dof.
      state_names = r.getStateFrame.coordinates(1:getNumDOF(r));
      obj.free_dof = find(~cellfun(@isempty,strfind(state_names,'arm')) + ...
                    ~cellfun(@isempty,strfind(state_names,'neck')));
      obj.con_dof = setdiff(1:getNumDOF(r),obj.free_dof)';
      
      input_names = r.getInputFrame.coordinates;
      obj.free_inputs = find(~cellfun(@isempty,strfind(input_names,'arm')) | ~cellfun(@isempty,strfind(input_names,'neck')));
      obj.con_inputs = setdiff(1:getNumInputs(r),obj.free_inputs)';
    else
      obj.free_dof = [];
      obj.con_dof = 1:getNumDOF(r);
      obj.free_inputs = [];
      obj.con_inputs = 1:getNumInputs(r);
    end
    
    
    obj.nu = getNumInputs(r);
    obj.nq = getNumDOF(r);
    if (~isfield(options,'R'))
      obj.R = 1e-6*eye(obj.nu);
    else
      typecheck(options.R,'double');
      sizecheck(options.R,[obj.nu,obj.nu]);
      obj.R = options.R;
    end
    
    if obj.solver==1 % use cplex
      obj.solver_options = cplexoptimset('cplex');
      obj.solver_options.diagnostics = 'on';
      obj.solver_options.maxtime = 0.001;
      % QP method: 
      %   0 	Automatic (default)
      %   1 	Primal Simplex
      %   2 	Dual Simplex
      %   3 	Network Simplex
      %   4 	Barrier
      %   5 	Sifting
      %   6 	Concurrent
      obj.solver_options.qpmethod = 4; 
      
    else % use gurobi
      obj.solver_options.outputflag = 0; % not verbose
      obj.solver_options.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
%       obj.solver_options.presolve = 0;

      if obj.solver_options.method == 2
        obj.solver_options.bariterlimit = 18; % iteration limit
        obj.solver_options.barhomogeneous = 0; % 0 off, 1 on
        obj.solver_options.barconvtol = 1e-4;
      end

    end    
  end
    
  function y=mimoOutput(obj,t,~,varargin)
    tic;
    q_ddot_des = varargin{1};
    supports = varargin{2};
    x = varargin{3};

    zmpd = getData(obj.zmpdata);
    r = obj.robot;

    %----------------------------------------------------------------------
    % Set up problem dimensions -------------------------------------------
    
    nd = 4; % for friction cone approx, hard coded for now
    dim = 3; % 3D
    nu = obj.nu;
    nq = obj.nq;
    nq_free = length(obj.free_dof); 
    nq_con = length(obj.con_dof); 
    nu_con = length(obj.con_inputs);     

    
    %----------------------------------------------------------------------
    % Compute kinematic and dynamic quantities ----------------------------
 
    q = x(1:nq); 
    qd = x(nq+(1:nq));
    kinsol = doKinematics(r,q,true);
    
    [H,C,B] = manipulatorDynamics(r,q,qd);
    
    H_con = H(obj.con_dof,:); 
    C_con = C(obj.con_dof);
    B_con = B(obj.con_dof,obj.con_inputs);
    
    if nq_free > 0
      H_free = H(obj.free_dof,:); 
      C_free = C(obj.free_dof);
      B_free = B(obj.free_dof,obj.free_inputs);
    end
    
    [xcom,J,dJ] = getCOM(r,kinsol);
    J = J(1:2,:); % only need COM x-y
    dJ = sparse(dJ(1:2,:)); % only need COM x-y
    Jdot = matGradMult(reshape(dJ,2*nq,nq),qd);

    active_supports = find(supports~=0);
    if (isempty(active_supports))
      warning('QPController::No supporting bodies...');
    end
    partial_supports = find(supports>0 & supports<1);
   
    % get active contacts
    [phi,Jz,D_] = contactConstraints(r,kinsol,active_supports);
    active_contacts = abs(phi)<0.005;
    nc = sum(active_contacts);

    if nc==0
      % ignore supporting body spec, use any body in contact
      [~,Jp,dJp] = contactPositions(r,kinsol);
      [phi,Jz,D_] = contactConstraints(r,kinsol);
      active_contacts = abs(phi)<0.005;
      nc = sum(active_contacts);
      partial_contacts = [];
      partial_idx = [];
    else
      % get support contact J, dJ for no-slip constraint
      [~,Jp,dJp] = contactPositions(r,kinsol,active_supports);
      partial_contacts = [];
      for i=active_supports'
        nC = size(getBodyContacts(r,i),2);
        if any(partial_supports==i)
          partial_contacts = [partial_contacts; ones(nC,1)];
        else
          partial_contacts = [partial_contacts; zeros(nC,1)];
        end
      end
      
      % get subset of active_contacts that are partial supports
      % NOTE: currently this is always an empty set because the partial contact
      % phase passes before the swing foot comes in  contact with the
      % ground. 
      partial_contacts = find(partial_contacts & active_contacts); 
      active_contacts = find(active_contacts);

      partial_idx = zeros(dim*length(partial_contacts),1);
      for i=1:length(partial_contacts);
        partial_idx((i-1)*dim+1:i*dim) = (partial_contacts(i)-1)*dim + (1:dim)';
      end
    end
    
    if nc > 0
      Jz = Jz(active_contacts,obj.con_dof); % only care about active contacts and constrained dofs

      active_idx = zeros(dim*length(active_contacts),1);
      for i=1:length(active_contacts);
        active_idx((i-1)*dim+1:i*dim) = (active_contacts(i)-1)*dim + (1:dim)';
      end
      Jp = Jp(active_idx,obj.con_dof); % only care about active contacts and constrained dofs
      Jpdot_ = reshape(dJp(active_idx,:),nc*dim*nq,nq);
      Jpdot = matGradMult(Jpdot_,qd);
      Jpdot = Jpdot(:,obj.con_dof);

      % D_ is the parameterization of the polyhedral approximation of the 
      %    friction cone, in joint coordinates (figure 1 from Stewart96)
      %    D{k}(i,:) is the kth direction vector for the ith contact (of nC)
      % Create Dbar such that Dbar(:,(k-1)*nd+i) is ith direction vector for 
      % the kth contact point
      D = cell(1,nc);
      for k=1:nc
        for i=1:nd
          D{k}(:,i) = D_{i}(active_contacts(k),obj.con_dof)'; 
        end
      end
      Dbar = [D{:}];
    end
    
    %----------------------------------------------------------------------
    % Linear inverted pendulum stuff --------------------------------------
        
    if zmpd.ti_flag
      S = zmpd.S;
      h = zmpd.h; 
      hddot = 0;
    else
      S = zmpd.S.eval(t);
      h = zmpd.h.eval(t); 
      hddot = zmpd.hddot.eval(t);
    end
    G = -h/(hddot+9.81)*eye(2); % zmp-input transfer matrix
    xlimp = [xcom(1:2); J*qd]; % state of LIP model

    
    %----------------------------------------------------------------------
    % Free DOF cost function ----------------------------------------------

    if nq_free > 0
      % approximate quadratic cost for free dofs with the appropriate matrix block
      Hqp_free = J(:,obj.free_dof)'*G'*obj.Qy*G*J(:,obj.free_dof) + obj.w*eye(nq_free);
      fqp_free = xlimp'*obj.F'*obj.Qy*G*J(:,obj.free_dof) + ...
                qd(obj.free_dof)'*Jdot(:,obj.free_dof)'*G'*obj.Qy*G*J(:,obj.free_dof) + ...
                xlimp'*S*obj.E*J(:,obj.free_dof) - obj.w*q_ddot_des(obj.free_dof)';

      % solve for qdd_free unconstrained
      qdd_free = -inv(Hqp_free)*fqp_free';
    end
        
    
    %----------------------------------------------------------------------
    % Build handy index matrices ------------------------------------------
    
    nf = nc+nc*nd; % number of contact force variables
    nparams = nq_con+nu_con+nf+nc*dim;
    Iqdd = zeros(nq_con,nparams); Iqdd(:,1:nq_con) = eye(nq_con);
    Iu = zeros(nu_con,nparams); Iu(:,nq_con+(1:nu_con)) = eye(nu_con);
    Iz = zeros(nc,nparams); Iz(:,nq_con+nu_con+(1:nc)) = eye(nc);
    Ibeta = zeros(nc*nd,nparams); Ibeta(:,nq_con+nu_con+nc+(1:nc*nd)) = eye(nc*nd);
    Ieps = zeros(nc*dim,nparams); 
    Ieps(:,nq_con+nu_con+nc+nc*dim+(1:nc*dim)) = eye(nc*dim);
    
    
    %----------------------------------------------------------------------
    % Set up problem constraints ------------------------------------------

    lb = [-1e3*ones(1,nq_con) r.umin(obj.con_inputs)' zeros(1,nf)   -obj.slack_limit*ones(1,nc*dim)]'; % qddot/input/contact forces/slack vars
    ub = [ 1e3*ones(1,nq_con) r.umax(obj.con_inputs)' 1e4*ones(1,nf) obj.slack_limit*ones(1,nc*dim)]';

    Aeq_ = cell(1,2);
    beq_ = cell(1,2);
    Ain_ = cell(1,nc);
    bin_ = cell(1,nc);

    % constrained dynamics
    if nc>0
      Aeq_{1} = H_con(:,obj.con_dof)*Iqdd - B_con*Iu - Jz'*Iz - Dbar*Ibeta;
    else
      Aeq_{1} = H_con(:,obj.con_dof)*Iqdd - B_con*Iu;
    end
    if nq_free > 0
      beq_{1} = -C_con - H_con(:,obj.free_dof)*qdd_free;
    else
      beq_{1} = -C_con;
    end
    
    if nc > 0
      % relative acceleration constraint
      Aeq_{2} = Jp*Iqdd + Ieps;
      beq_{2} = -Jpdot*qd(obj.con_dof) - 1.0*Jp*qd(obj.con_dof);

      % linear friction constraints
      % TEMP: hard code mu
      mu = 1.0*ones(nc,1);
      for i=1:nc
        Ain_{i} = -mu(i)*Iz(i,:) + sum(Ibeta((i-1)*nd+(1:nd),:));
        bin_{i} = 0;
      end
    end
    
    % linear equality constraints: Aeq*alpha = beq
    Aeq = sparse(vertcat(Aeq_{:}));
    beq = vertcat(beq_{:});
      
    % linear inequality constraints: Ain*alpha <= bin
    Ain = sparse(vertcat(Ain_{:}));
    bin = vertcat(bin_{:});

    
    %----------------------------------------------------------------------
    % QP cost function ----------------------------------------------------
    %
    %  min: quad(F*x+G*(Jdot*qd + J*qdd),Q) + 2*x'*S*(A*x + E*(Jdot*qd + J*qdd)) + w*quad(qddot_ref - qdd) + quad(u,R) + quad(epsilon)
    
    Hqp = repmat(eye(nparams),2,1)'*vertcat(Iqdd'*J(:,obj.con_dof)'*G'*obj.Qy*G*J(:,obj.con_dof)*Iqdd, obj.w*Iqdd'*Iqdd);
    
    fqp = horzcat(xlimp'*obj.F'*obj.Qy*G*J(:,obj.con_dof)*Iqdd, ...
          qd(obj.con_dof)'*Jdot(:,obj.con_dof)'*G'*obj.Qy*G*J(:,obj.con_dof)*Iqdd, ...
          xlimp'*S*obj.E*J(:,obj.con_dof)*Iqdd, ...
          -obj.w*q_ddot_des(obj.con_dof)'*Iqdd)*repmat(eye(nparams),4,1);

    % quadratic input cost
    Hqp(nq_con+(1:nu_con),nq_con+(1:nu_con)) = obj.R(obj.con_inputs,obj.con_inputs);

    % add cost term for transitional contacts
    qz = zeros(nc,1);
    qz(partial_contacts) = 1e-6*ones(length(partial_contacts),1);
    qbeta = zeros(nc*nd,1);
    qbeta(partial_idx) = 1e-6*ones(length(partial_idx),1);
    Hqp(nq_con+nu_con+(1:nc),nq_con+nu_con+(1:nc)) = diag(qz);
    Hqp(nq_con+nu_con+nc+(1:nc*nd),nq_con+nu_con+nc+(1:nc*nd)) = diag(qbeta);
    
    % quadratic slack var cost 
    Hqp(nparams-nc*dim+1:end,nparams-nc*dim+1:end) = eye(nc*dim); 

    %----------------------------------------------------------------------
    % Solve QP ------------------------------------------------------------
        
    if obj.solver==1
      alpha = quadprog(Hqp,fqp,Ain,bin,Aeq,beq,lb,ub);
      % CURRENTLY CRASHES MATLAB ON MY MACHINE -sk
      %alpha = cplexqp(Hqp,fqp,Ain,bin,Aeq,beq,lb,ub,[],obj.solver_options);
    
    else
      model.Q = sparse(Hqp);
      model.obj = 2*fqp;
      model.A = [Aeq; Ain];
      model.rhs = [beq; bin];
      model.sense = [repmat('=',length(beq),1); repmat('<',length(bin),1)];
      model.lb = lb;
      model.ub = ub;

%       tic;
      result = gurobi(model,obj.solver_options);
%       toc
      alpha = result.x;
%       dvals = result.pi;
    end
    
    %----------------------------------------------------------------------
    % Solve for free inputs -----------------------------------------------
    if nq_free > 0
      qdd = zeros(nq,1);
      qdd(obj.free_dof) = qdd_free;
      qdd(obj.con_dof) = alpha(1:nq_con);

      u_free = B_free\(H_free*qdd + C_free);
      u = zeros(nu,1);
      u(obj.free_inputs) = u_free;
      u(obj.con_inputs) = alpha(nq_con+(1:nu_con));

      % saturate inputs
      y = max(r.umin,min(r.umax,u));
    else
      y = alpha(nq+(1:nu));
    end
    toc
   
  end
  end

  properties
    robot % to be controlled
    zmpdata
    w = 1.0; % objective function weight
    slack_limit = 1.0; % maximum absolute magnitude of acceleration slack variable values
    free_dof % dof for which we perform unconstrained minimization (i.e., dofs not in the kinematic chain to contact points)
    con_dof 
    free_inputs
    con_inputs
    nq
    nu
    R  % quadratic input cost matrix
    % LIP stuff
    A = [zeros(2),eye(2); zeros(2,4)]; % state transfer matrix
    E = [zeros(2); eye(2)]; % input transfer matrix
    F = [eye(2),zeros(2)]; % zmp-state transfer matrix
    Qy = eye(2); % output cost matrix--must match ZMP LQR cost 
    solver = 0; % 0: gurobi, 1:cplex
    solver_options = struct();
  end
end
