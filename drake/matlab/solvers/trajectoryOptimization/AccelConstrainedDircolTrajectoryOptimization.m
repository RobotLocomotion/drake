classdef AccelConstrainedDircolTrajectoryOptimization < DircolTrajectoryOptimization
  % Generic implementation of the DIRCON algorithm found in
  % "Optimization and stabilization of trajectories for constrained
  %  dynamical systems" by Posa, Kuindersma and Tedrake 2016.
  % 
  % This is a direct collocation based algorithm that handles kinematic
  % constraints, using Drake's position constraint functionality.
  % 
  % Extracts these constraints automatically and incorporates them into
  % the optimization.
  %
  % Use the constructor:
  % obj = AccelConstrainedDircolTrajectoryOptimization(plant,N,duration,options)
  % which accepts the default Dircol arguments as well as some additional
  % options:
  %   options.active_inds : A subset of the position constraints to
  %     incorproate--note ALL constraint forces are always active. This
  %     option is primarily for eliminating redundant constraints, but not
  %     their associated forces.  Default = all
  %   options.relative_constraints : Can be either a single boolean or a
  %     vector of boolean corresponding to the active cosntraints. If true,
  %     adds a slack variable to treat constraints as having no *absolute* 
  %     true value. phi(q_i) = phi(q_j) instead of phi(q_i) = 0. Defaults
  %     to false.
  %   options.constrain_start : Constraint for i=1, Defaults to true.
  %   options.constrain_end : Constraint for i=N, Defaults to true.
  %   options.constrain_phi_start : Only active if constrain_start is true.
  %     Controls whether phi(q_1) is constrained. If false, constraints
  %     phidot only.
  %   options.constrain_phi_end : see constrain_phi_start
  %   options.accel_cost : Adds a cost term equal to sum(accel_cost *
  %   qddot^2).  This is useful for as a small regularizing term. Defaults
  %   to 0.
  
  
  properties
    l_inds       % Indices into the knot point contact forces
    lc_inds      % Indices into the collocation point contact forces (accel slack)
    vc_inds      % Indices into the collocation point position slack
    cstrval_inds % indices into position constraint offsets
    cstr_to_val_map  % maps relative constraint indices to their respective variables above
    nC  % total number of constraints
    active_inds % subset of the plants constraints that are active
  end
  
  methods
    function obj = AccelConstrainedDircolTrajectoryOptimization(plant,N,duration,options)
      if nargin < 4
        options = struct();
      end      
            
      if ~isfield(options,'active_inds')
        options.active_inds = 1:plant.num_position_constraints;
      end
      
      nActive = length(options.active_inds);
      
      if ~isfield(options,'relative_constraints')
        options.relative_constraints = false(nActive,1);
      elseif length(options.relative_constraints) == 1
        options.relative_constraints = repmat(options.relative_constraints,nActive,1);
      end      
      
      if ~isfield(options,'constrain_start')
        options.constrain_start = true;
      end
      
      if ~isfield(options,'constrain_end')
        options.constrain_end = true;
      end
      
      if ~isfield(options,'constrain_phi_start')
        options.constrain_phi_start = true;
      end
      
      if ~isfield(options,'constrain_phi_end')
        options.constrain_phi_end = true;
      end      
      
      if ~isfield(options,'accel_cost')
        options.accel_cost = 0;
      end
      
      obj = obj@DircolTrajectoryOptimization(plant,N,duration,options);
    end
        
    function obj = addDynamicConstraints(obj)
      % Main function that generates constraints for the optimization
      % program. Summary of constraints added:
      %  - collocation constraints for the dynamics between knot points
      %  - phi,phidot constraints at the knot points
      %  - phiddot constraints at the knot points
      %  - (optional) qddot cost at the knot points
      N = obj.N;
      nq = obj.plant.getNumPositions();
      nv = obj.plant.getNumVelocities();
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
      nActive = length(obj.active_inds);
      dyn_constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + 2*nU + 4*nC + 1;
      cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.constraint_fun);
      cnstr = setName(cnstr,'collocation');

      % create shared data functions to calculate dynamics at the knot
      % points
      shared_data_index = obj.getNumSharedDataFunctions;
      for i=1:obj.N,
        obj = obj.addSharedDataFunction(@obj.dynamics_data,{obj.x_inds(:,i);obj.u_inds(:,i);obj.l_inds(:,i)});
      end
                

      for i=1:obj.N-1,
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1);
          obj.l_inds(:,i); obj.l_inds(:,i+1); obj.lc_inds(:,i); obj.vc_inds(:,i)};
        dyn_constraints{i} = cnstr;
        
        obj = obj.addConstraint(dyn_constraints{i}, dyn_inds{i},[shared_data_index+i;shared_data_index+i+1]);
      end
      
      if obj.nC > 0
        for i=1:obj.N,
          phiddot_cnstr = FunctionHandleConstraint(zeros(nC,1),zeros(nC,1),nX+nU+nC,@obj.phiddot_constraint);
          obj = obj.addConstraint(phiddot_cnstr,{obj.x_inds(:,i);obj.u_inds(:,i);obj.l_inds(:,i)},shared_data_index+i);
        end
      end

      if obj.nC > 0        
        noffset = length(obj.cstrval_inds);
        pos_cnstr = FunctionHandleConstraint(zeros(2*nActive,1),zeros(2*nActive,1),nX+noffset,@(x,offset,data) obj.position_constraint_fun(x,offset,false,data));
        cnstr_sparsity = [ones(nActive,nq) zeros(nActive,nv) ones(nActive,noffset); ones(nActive,nX) zeros(nActive,noffset)];
        [cnstr_i, cnstr_j] = ind2sub([2*nActive, nX+noffset],find(cnstr_sparsity));
        pos_cnstr = pos_cnstr.setSparseStructure(cnstr_i,cnstr_j);
        
        phidot_cnstr = FunctionHandleConstraint(zeros(nActive,1),zeros(nActive,1),nX,@(x,data) obj.position_constraint_fun(x,[],true,data));
        
        if obj.options.constrain_start,
          if obj.options.constrain_phi_start
            obj = obj.addConstraint(pos_cnstr, {obj.x_inds(:,1),obj.cstrval_inds},shared_data_index+1);
          else
            obj = obj.addConstraint(phidot_cnstr, {obj.x_inds(:,1)},shared_data_index+1);
          end
        end
        for i=2:obj.N-1,
          obj = obj.addConstraint(pos_cnstr, {obj.x_inds(:,i),obj.cstrval_inds},shared_data_index+i);
        end
        if obj.options.constrain_end,
          if obj.options.constrain_phi_end
            obj = obj.addConstraint(pos_cnstr, {obj.x_inds(:,N),obj.cstrval_inds},shared_data_index+N);
          else
            obj = obj.addConstraint(phidot_cnstr, {obj.x_inds(:,N)},shared_data_index+N);
          end
        end
      end
      
      
      if obj.options.accel_cost > 0
        accel_cost = FunctionHandleObjective(nX+nU+nC,@obj.accel_cost_fun);
        for i=1:obj.N,
          obj = obj.addCost(accel_cost,{obj.x_inds(:,i);obj.u_inds(:,i);obj.l_inds(:,i)},shared_data_index+i);
        end
        
      end
    end
    
    function [f,df] = accel_cost_fun(obj,x,u,lambda,data)
      % Cost function on qddot^2
      nq = obj.plant.getNumPositions;
      qddot = data.xdot(nq+1:end);
      dqddot = data.dxdot(nq+1:end,2:end);
      f = obj.options.accel_cost*(qddot'*qddot);
      df = 2*obj.options.accel_cost*qddot'*dqddot;
    end
    
    function [f,df] = constraint_fun(obj,h,x0,x1,u0,u1,lambda0,lambda1,lambdac,vc,data0,data1)
      % calculate xdot at knot points
      %  [xdot0,dxdot0] = obj.plant.dynamics(0,x0,u0);
      %  [xdot1,dxdot1] = obj.plant.dynamics(0,x1,u1);
      
      nq = obj.plant.getNumPositions;
      nv = obj.plant.getNumVelocities();
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
      
      % use the shared data objects for the dynamics at the knot points
      xdot0 = data0.xdot;
      dxdot0 = data0.dxdot;
      xdot1 = data1.xdot;
      dxdot1 = data1.dxdot;
      
      % cubic interpolation to get xcol and xdotcol, as well as
      % derivatives
      xcol = .5*(x0+x1) + h/8*(xdot0-xdot1);
      dxcol = [1/8*(xdot0-xdot1) (.5*eye(nX) + h/8*dxdot0(:,2:1+nX)) ...
        (.5*eye(nX) - h/8*dxdot1(:,2:1+nX)) h/8*dxdot0(:,nX+2:1+nX+nU) -h/8*dxdot1(:,nX+2:1+nX+nU) ...
        h/8*dxdot0(:,nX+nU+2:1+nX+nU+nC) -h/8*dxdot1(:,nX+nU+2:1+nX+nU+nC) zeros(nX,2*nC)];
      xdotcol = -1.5*(x0-x1)/h - .25*(xdot0+xdot1);
      dxdotcol = [1.5*(x0-x1)/h^2 (-1.5*eye(nX)/h - .25*dxdot0(:,2:1+nX)) ...
        (1.5*eye(nX)/h - .25*dxdot1(:,2:1+nX)) -.25*dxdot0(:,nX+2:1+nX+nU) -.25*dxdot1(:,nX+2:1+nX+nU) ...
        -.25*dxdot0(:,nX+nU+2:1+nX+nU+nC) -.25*dxdot1(:,nX+nU+2:1+nX+nU+nC) zeros(nX,2*nC)];
      
      % evaluate xdot at xcol, using foh on control input
      datac = obj.dynamics_data(xcol,.5*(u0+u1),lambdac);
      g = datac.xdot + [datac.J'*vc; zeros(nv,1)];
      if nC > 0
        dgdxcol = [datac.dxdot zeros(nX,nC)] + [zeros(nq,1) matGradMult(datac.dJ,vc,true) zeros(nq,nv+nU+nC) datac.J'; zeros(nq,size(datac.dxdot,2) + nC)];
      else
        dgdxcol = datac.dxdot;
      end
      dg = dgdxcol(:,2:1+nX)*dxcol + [zeros(nX,1+2*nX) .5*dgdxcol(:,2+nX:1+nX+nU) .5*dgdxcol(:,2+nX:1+nX+nU) zeros(nX,2*nC) dgdxcol(:,nX+nU+2:1+nX+nU+2*nC)];
      
      % constraint is the difference between the two
      f = xdotcol - g;
      df = dxdotcol - dg;
      
      
      df = [df(:,1)*h + f, df(:,2:end)*h];
      f = f*h;
    end
    
    function [f,df] = phiddot_constraint(obj,x,u,lambda,data)
      % A constraint on phiddot
      % Uses values computed in dynamcis_data
      f = data.phiddot;
      df = data.dphiddot(:,2:end);
    end
    
    function [f,df] = position_constraint_fun(obj,x,offset,phidot_only,data)
      % A constraint on phi and phidot
      %  phi(q) = offset
      %  phidot(q,qdot) = 0
      % Uses values computed in dynamcis_data
      nq = obj.plant.getNumPositions;
      nActive = length(obj.active_inds);
      q = x(1:nq);
      qd = x(nq+1:end);
      
      phi = data.phi(obj.active_inds,:);
      J = data.J(obj.active_inds,:);
      Jdot = data.Jdot(obj.active_inds,:);
      
      noffset = length(offset);
      if ~phidot_only
        f = [phi;J*qd];
        f(obj.cstr_to_val_map) = f(obj.cstr_to_val_map) - offset; 
        df = [J zeros(nActive,nq+noffset);
          Jdot J zeros(nActive,noffset)];
        df(obj.cstr_to_val_map,end-noffset+1:end) = -eye(noffset);
      else
        f = J*qd;
        df = [Jdot J zeros(nActive,noffset)];
      end     
    end
    
    function data = dynamics_data(obj,x,u,lambda)
      % The bulk of the computation is performed here and cached for later
      % At knot points, computes:
      %   xdot
      %   phi
      %   J=dphi/dq
      %   Jdot = dJ/dt
      %   Jdotqqd = Jdot*qd
      %   dJotqd = [d/dx d/du d/dlambda]Jdotqd
      %   phiddot = d^2/dt^2 phi
      %   dphiddot = [d/dx d/du d/dlambda]phiddot
      nq = obj.plant.getNumPositions;
      nC = obj.nC;
      nU = obj.plant.getNumInputs();

      q = x(1:nq);
      v = x(nq+1:end);
      
      [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics(q,v);
      Hinv = inv(H);
      if nC > 0      
        [phi,J,dJ,Jdotqd,dJdotqd] = obj.plant.positionConstraintsWithJdot(q,v);
        dJ = reshape(dJ,[],nq);
        Jdot = matGradMult(dJ,v);      
        vdot = Hinv*(B*u - C + J'*lambda);
        dtau = matGradMult(dB,u) - dC + [matGradMult(dJ,lambda,true), zeros(nq,nq)];

        dvdot = [zeros(nq,1),...
          -Hinv*matGradMult(dH(:,1:nq),vdot) + Hinv*dtau(:,1:nq),...
          +Hinv*dtau(:,1+nq:end), Hinv*B, Hinv*J'];

        kinsol = obj.plant.doKinematics(q,v,struct('compute_gradients',true));
        [VqInv,dVqInv] = obj.plant.vToqdot(kinsol);
        xdot = [VqInv*v;vdot];
        dxdot = [...
          zeros(nq,1), matGradMult(dVqInv, v), VqInv, zeros(nq,nU+nC); dvdot];   

        data.xdot = xdot;
        data.dxdot = dxdot;
        data.phi = phi;
        data.J = J;
        data.dJ = dJ;
        data.Jdot = Jdot;  
        data.Jdotqd = Jdotqd;
        data.dJdotqd = dJdotqd;
        data.phiddot = Jdotqd + J*vdot;
        data.dphiddot = [zeros(nC,1) (dJdotqd + [matGradMult(dJ,vdot) zeros(nC,nq)]) zeros(nC,nU+nC)] + J*dvdot;
        
      else
        vdot = Hinv*(B*u - C);
        dtau = matGradMult(dB,u) - dC;
        
        dvdot = [zeros(nq,1),...
          -Hinv*matGradMult(dH(:,1:nq),vdot) + Hinv*dtau(:,1:nq),...
          +Hinv*dtau(:,1+nq:end), Hinv*B];
        
        [VqInv,dVqInv] = obj.plant.vToqdot(q);
        xdot = [VqInv*v;vdot];
        dxdot = [...
          zeros(nq,1), matGradMult(dVqInv, v), VqInv, zeros(nq,nU+nC);
          dvdot];
        
        data.xdot = xdot;
        data.dxdot = dxdot;
        data.phi = [];
        data.J = zeros(0,nq);
        data.dJ = zeros(0,nq);
        data.Jdot = zeros(0,nq);
        data.Jdotqd = zeros(0,1);
        data.dJdotqd = zeros(0,length(x));
        data.phiddot = zeros(0,1);
        data.dphiddot = zeros(0,length(x) + nU);
      end      
    end
    
    function obj = setupVariables(obj,N)
      % Adds appropriate variables to NonlinearProgram and stores their
      % indices
      obj = setupVariables@DircolTrajectoryOptimization(obj,N);
            
      nActive = length(obj.active_inds);
      nC = obj.plant.num_position_constraints;
      obj.nC = nC;
      obj.active_inds = obj.options.active_inds;
      num_vars = N*nC + 2*(N-1)*nC;

      obj.l_inds = reshape(obj.num_vars + (1:nC*N),nC,N);
      obj.lc_inds = reshape(obj.num_vars + nC*N + (1:nC*(N-1)),nC,N-1);
      obj.vc_inds = reshape(obj.num_vars + nC*N + (N-1)*nC + (1:nC*(N-1)),nC,N-1);
     
      obj = obj.addDecisionVariable(num_vars);
      
      if any(obj.options.relative_constraints)
        obj.cstr_to_val_map = find(obj.options.relative_constraints(obj.active_inds));
      else
        obj.cstr_to_val_map = zeros(0,1);
      end
      num_rel_constraints = length(obj.cstr_to_val_map);
      obj.cstrval_inds = (1:num_rel_constraints)' + obj.num_vars;
      
      obj = obj.addDecisionVariable(num_rel_constraints);
    end
    
    function [xtraj,utraj,ltraj,z,F,info,infeasible_constraint_name] = solveTrajFromZ(obj,z0)
      % Solve given an initial value for all variables, z0
      [xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTrajFromZ@DircolTrajectoryOptimization(obj,z0);
      ltraj = obj.reconstructForceTrajectory(z);
    end
    
    function [xtraj,utraj,ltraj,z,F,info,infeasible_constraint_name] = solveTraj(obj,varargin)
      % Default solve method, akin to parent classes.
      [xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTraj@DircolTrajectoryOptimization(obj,varargin{:});            
      ltraj = obj.reconstructForceTrajectory(z);
    end
    
    function ltraj = reconstructForceTrajectory(obj,z)
      % Piecewise linear interpretation of forces. Use collocation forces
      % at collocation points, so a total of 2N-1 points.
      t_l = zeros(obj.N*2-1,1);
      lambda = zeros(size(obj.l_inds,1),obj.N*2 -1);
      lambda(:,1:2:end) = z(obj.l_inds);
      lambda(:,2:2:end) = z(obj.lc_inds);
      t_x = [0; cumsum(z(obj.h_inds))];
      t_l(1:2:end) = t_x;
      t_l(2:2:end) = (t_x(1:end-1) + t_x(2:end))/2;
      if obj.nC > 0
        ltraj = PPTrajectory(foh(t_l,lambda));
      else
        ltraj = [];
      end
    end
    
    
    function xtraj = reconstructStateTrajectory(obj,z)
      % Interpolate between knot points to reconstruct a trajectory using
      % the hermite spline
      t = [0; cumsum(z(obj.h_inds))];
      u = reshape(z(obj.u_inds),[],obj.N);
      
      x = reshape(z(obj.x_inds),[],obj.N);
      lambda = reshape(z(obj.l_inds),[],obj.N);
      xdot = zeros(size(x,1),obj.N);
      for i=1:obj.N,
        xdot(:,i) = obj.constrainedDynamics(t(i),x(:,i),u(:,i),lambda(:,i));
%         xdot(:,i) = obj.plant.dynamics(t(i),x(:,i),u(:,i));
      end
      xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
    end
    
    function xdot = constrainedDynamics(obj,t,x,u,lambda)
      % xdot = constrainedDynamics(obj,t,x,u,lambda)
      %  Compute the constrained manipulator dynamics
      %   qddot  = inv(H)*(-C + B*u + J^T*lambda)
      nq = obj.plant.getNumPositions;
      nU = obj.plant.getNumInputs();

      q = x(1:nq);
      v = x(nq+1:end);
      
      [H,C,B] = obj.plant.manipulatorDynamics(q,v);
      [~,J] = obj.plant.positionConstraints(q);     
      
      if ~isempty(lambda)
        vdot = H\(B*u - C + J'*lambda);
      else
        vdot = H\(B*u - C);
      end
      xdot = [v;vdot];
    end    
    
    function z0 = getInitialVars(obj,t_init,traj_init)
      % evaluates the initial trajectories at the sampled times and
      % constructs the nominal z0. Overwrite to implement in a different
      % manner
      if isscalar(t_init)
        t_init = linspace(0,t_init,obj.N);
      elseif length(t_init) ~= obj.N
        error('The initial sample times must have the same length as property N')
      end
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(t_init);
      
      if nargin<3, traj_init = struct(); end
      
      nU = getNumInputs(obj.plant);
      if isfield(traj_init,'u')
        z0(obj.u_inds) = traj_init.u.eval(t_init);
      else
        z0(obj.u_inds) = 0.01*randn(nU,obj.N);
      end
      
      if isfield(traj_init,'l')
        z0(obj.l_inds) = traj_init.l.eval(t_init);
        t_c = (t_init(1:end-1) + t_init(2:end))/2;
        z0(obj.lc_inds) = traj_init.l.eval(t_c);
      end
            
      if isfield(traj_init,'x')
        z0(obj.x_inds) = traj_init.x.eval(t_init);
      else
        if nU>0
          if ~isfield(traj_init,'u')
            traj_init.u = setOutputFrame(PPTrajectory(foh(t_init,reshape(z0(obj.u_inds),nU,obj.N))),getInputFrame(obj.plant));
          end
          
          % todo: if x0 and xf are equality constrained, then initialize with
          % a straight line from x0 to xf (this was the previous behavior)
          
          %simulate
          sys_ol = cascade(traj_init.u,obj.plant);
        else
          sys_ol = obj.plant;
        end
        
        if ~isfield(traj_init,'x0')
          [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)]);
        else
          [~,x_sim] = sys_ol.simulate([t_init(1) t_init(end)],traj_init.x0);
        end
        
        z0(obj.x_inds) = x_sim.eval(t_init);
      end
    end
  end
end