classdef ConstrainedDircolTrajectoryOptimization < DircolTrajectoryOptimization
  % Direct colocation approach
  % Over each interval, f(x(k),u(k)) and f(x(k+1),u(k+1)) are evaluated to
  % determine d/dt x(k) and d/dt x(k+1). A cubic spline is fit over the
  % interval x and d/dt x at the end points.
  % x(k+.5) and d/dt x(k+.5) are determined based on this spline.
  % Then, the dynamics constraint is:
  % d/dt x(k+.5) = f(x(k+.5),.5*u(k) + .5*u(k+1))
  %
  %  integrated cost is: .5*h(1)*g(x(1),u(1)) + .5*h(N-1)*g(x(N),u(N)) +
  %                   sum((.5*h(i)+.5*h(i-1))*g(x(i),u(i))
  %  more simply stated, integrated as a zoh with half of each time
  %  interval on either side of the knot point
  % this might be the wrong thing for the cost function...
  properties
    l_inds
    lc_inds
    cstrval_inds
    cstr_to_val_map  % maps relative constraint indices to their respective variables above
    nC
    active_inds
  end
  
  methods
    function obj = ConstrainedDircolTrajectoryOptimization(plant,N,duration,options)
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
      
      if ~isfield(options,'lambda_bound')
        options.lambda_bound = 100;
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
      
      if ~isfield(options,'test_bound')
        options.test_bound = false;
      end
%       if ~isfield(options,'start_active_inds')
%         options.start_active_inds = options.active_inds;
%       end
      
      obj = obj@DircolTrajectoryOptimization(plant,N,duration,options);
    end
    
    function [f,df] = phidotConstraint(obj,h,x0,x1,u0,u1,lambda0,lambda1,data0,data1)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
      nq = obj.plant.getNumPositions; 
      
      % use the shared data objects for the dynamics at the knot points
      xdot0 = data0.xdot;
      dxdot0 = data0.dxdot;
      xdot1 = data1.xdot;
      dxdot1 = data1.dxdot;
      
      % get state at t=h/4
      % x(h/4) = 27/32*x0 + 5/32*x1 + 9/64*h*xd0 - 3/64*h*xd1
      % xd(h/4) = 3/16*xd0 - 5/16*xd1 + 9/(8h)*(x1 - x0)
      xc = 27/32*x0 + 5/32*x1 + 9/64*h*xdot0 - 3/64*h*xdot1;
      dxc = [9/64*xdot0-3/64*xdot1, 27/32*eye(nX)+9/64*h*dxdot0(:,2:1+nX), 5/32*eye(nX)-3/64*h*dxdot1(:,2:1+nX), ...
        9/64*h*dxdot0(:,2+nX:1+nX+nU), -3/64*h*dxdot1(:,2+nX:1+nX+nU), 9/64*h*dxdot0(:,nX+nU+2:1+nX+nU+nC),...
        -3/64*h*dxdot1(:,nX+nU+2:1+nX+nU+nC)];
      
      qc = xc(1:nq);
      vc = xc(nq+1:end);
      [~,J,dJ] = obj.plant.positionConstraints(qc);
      J = J(obj.active_inds,:);      
      dJ = dJ(obj.active_inds,:);
      dJ = reshape(dJ,[],nq);
      Jdot = matGradMult(dJ,vc);
      phidot = J*vc;
      dphidot = [Jdot J];
      
      C = obj.options.lambda_bound;
      f = [phidot + C*h^3;-phidot + C*h^3]*(obj.N^3/C);
      df = [dphidot*dxc;-dphidot*dxc];
      df(:,1) = df(:,1)+3*C*h^2*ones(length(f),1);
      df = df*(obj.N^3/C);      
      
      % should this be h^4? try adding as a cost insteads
%       f = [phidot;phidot];
%       df = [dphidot*dxc;dphidot*dxc];
    end   
    
    function [f,df] = phidotBounded(obj,h,x0,x1,u0,u1,lambda0,lambda1,bound,data0,data1)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
      nq = obj.plant.getNumPositions;
      
      % use the shared data objects for the dynamics at the knot points
      xdot0 = data0.xdot;
      dxdot0 = data0.dxdot;
      xdot1 = data1.xdot;
      dxdot1 = data1.dxdot;
      
      % get state at t=h/4
      % x(h/4) = 27/32*x0 + 5/32*x1 + 9/64*h*xd0 - 3/64*h*xd1
      % xd(h/4) = 3/16*xd0 - 5/16*xd1 + 9/(8h)*(x1 - x0)
      xc = 27/32*x0 + 5/32*x1 + 9/64*h*xdot0 - 3/64*h*xdot1;
      dxc = [9/64*xdot0-3/64*xdot1, 27/32*eye(nX)+9/64*h*dxdot0(:,2:1+nX), 5/32*eye(nX)-3/64*h*dxdot1(:,2:1+nX), ...
        9/64*h*dxdot0(:,2+nX:1+nX+nU), -3/64*h*dxdot1(:,2+nX:1+nX+nU), 9/64*h*dxdot0(:,nX+nU+2:1+nX+nU+nC),...
        -3/64*h*dxdot1(:,nX+nU+2:1+nX+nU+nC)];
      
      qc = xc(1:nq);
      vc = xc(nq+1:end);
      [~,J,dJ] = obj.plant.positionConstraints(qc);
      J = J(obj.active_inds,:);
      dJ = dJ(obj.active_inds,:);
      dJ = reshape(dJ,[],nq);
      Jdot = matGradMult(dJ,vc);
      phidot = J*vc;
      dphidot = [Jdot J];
      
      C = obj.options.lambda_bound;
      f = [phidot + bound;-phidot + bound];
      df = [[dphidot*dxc;-dphidot*dxc] ones(length(f),1)];
      
      % should this be h^4? try adding as a cost insteads
      %       f = [phidot;phidot];
      %       df = [dphidot*dxc;dphidot*dxc];
    end
    
    function [f,df] = phidotSquared(obj,h,x0,x1,u0,u1,lambda0,lambda1,data0,data1)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
      nq = obj.plant.getNumPositions; 
      
      % use the shared data objects for the dynamics at the knot points
      xdot0 = data0.xdot;
      dxdot0 = data0.dxdot;
      xdot1 = data1.xdot;
      dxdot1 = data1.dxdot;
      
      % get state at t=h/4
      % x(h/4) = 27/32*x0 + 5/32*x1 + 9/64*h*xd0 - 3/64*h*xd1
      % xd(h/4) = 3/16*xd0 - 5/16*xd1 + 9/(8h)*(x1 - x0)
      xc = 27/32*x0 + 5/32*x1 + 9/64*h*xdot0 - 3/64*h*xdot1;
      dxc = [9/64*xdot0-3/64*xdot1, 27/32*eye(nX)+9/64*h*dxdot0(:,2:1+nX), 5/32*eye(nX)-3/64*h*dxdot1(:,2:1+nX), ...
        9/64*h*dxdot0(:,2+nX:1+nX+nU), -3/64*h*dxdot1(:,2+nX:1+nX+nU), 9/64*h*dxdot0(:,nX+nU+2:1+nX+nU+nC),...
        -3/64*h*dxdot1(:,nX+nU+2:1+nX+nU+nC)];
      
      qc = xc(1:nq);
      vc = xc(nq+1:end);
      [~,J,dJ] = obj.plant.positionConstraints(qc);
      J = J(obj.active_inds,:);      
      dJ = dJ(obj.active_inds,:);
      dJ = reshape(dJ,[],nq);
      Jdot = matGradMult(dJ,vc);
      phidot = J*vc;
      dphidot = [Jdot J];
      
      f = phidot'*phidot*1e4;
      df = [2*phidot'*dphidot*dxc]*1e4;
    end
    
    function obj = addDynamicConstraints(obj)
      N = obj.N;
      nq = obj.plant.getNumPositions();
      nv = obj.plant.getNumVelocities();
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
      nActive = length(obj.active_inds);
      dyn_constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      
      n_vars = 2*nX + 2*nU + 3*nC + 1;
      cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.constraint_fun);
      cnstr = setName(cnstr,'collocation');

      % create shared data functions to calculate dynamics at the knot
      % points
      shared_data_index = obj.getNumSharedDataFunctions;
      for i=1:obj.N,
        obj = obj.addSharedDataFunction(@obj.dynamics_data,{obj.x_inds(:,i);obj.u_inds(:,i);obj.l_inds(:,i)});
      end
      
      if obj.options.test_bound
        obj = obj.addDecisionVariable(1);
        bound_ind = obj.num_vars;
        obj = obj.addCost(LinearConstraint(0,0,1),bound_ind);
      end                

      for i=1:obj.N-1,
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1);
          obj.l_inds(:,i); obj.l_inds(:,i+1); obj.lc_inds(:,i)};
        dyn_constraints{i} = cnstr;
        
        obj = obj.addConstraint(dyn_constraints{i}, dyn_inds{i},[shared_data_index+i;shared_data_index+i+1]);
        
        if obj.nC > 0
          
          if ~obj.options.test_bound 
            phidotcon = FunctionHandleConstraint(zeros(2*nActive,1),inf(2*nActive,1),n_vars-nC,@obj.phidotConstraint);
            obj = obj.addConstraint(phidotcon, dyn_inds{i}(1:end-1),[shared_data_index+i;shared_data_index+i+1]);
          else
            phidotcon = FunctionHandleConstraint(zeros(2*nActive,1),inf(2*nActive,1),n_vars-nC+1,@obj.phidotBounded);
            obj = obj.addConstraint(phidotcon, [dyn_inds{i}(1:end-1);bound_ind],[shared_data_index+i;shared_data_index+i+1]);
          end
                
%           phidotcost = FunctionHandleConstraint(0,0,n_vars-nC,@obj.phidotSquared);
%           obj = obj.addCost(phidotcost, dyn_inds{i}(1:end-1),[shared_data_index+i;shared_data_index+i+1]);


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
    end
    
    function [f,df] = constraint_fun(obj,h,x0,x1,u0,u1,lambda0,lambda1,lambdac,data0,data1)
      % calculate xdot at knot points
      %  [xdot0,dxdot0] = obj.plant.dynamics(0,x0,u0);
      %  [xdot1,dxdot1] = obj.plant.dynamics(0,x1,u1);
      
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
        h/8*dxdot0(:,nX+nU+2:1+nX+nU+nC) -h/8*dxdot1(:,nX+nU+2:1+nX+nU+nC) zeros(nX,nC)];
      xdotcol = -1.5*(x0-x1)/h - .25*(xdot0+xdot1);
      dxdotcol = [1.5*(x0-x1)/h^2 (-1.5*eye(nX)/h - .25*dxdot0(:,2:1+nX)) ...
        (1.5*eye(nX)/h - .25*dxdot1(:,2:1+nX)) -.25*dxdot0(:,nX+2:1+nX+nU) -.25*dxdot1(:,nX+2:1+nX+nU) ...
        -.25*dxdot0(:,nX+nU+2:1+nX+nU+nC) -.25*dxdot1(:,nX+nU+2:1+nX+nU+nC) zeros(nX,nC)];
      
      % evaluate xdot at xcol, using foh on control input
      datac = obj.dynamics_data(xcol,.5*(u0+u1),lambdac);
      g = datac.xdot;
      dgdxcol = datac.dxdot;
      dg = dgdxcol(:,2:1+nX)*dxcol + [zeros(nX,1+2*nX) .5*dgdxcol(:,2+nX:1+nX+nU) .5*dgdxcol(:,2+nX:1+nX+nU) zeros(nX,2*nC) dgdxcol(:,nX+nU+2:1+nX+nU+nC)];
      
      % constraint is the difference between the two
      f = xdotcol - g;
      df = dxdotcol - dg;
      
      
      df = [df(:,1)*h + f, df(:,2:end)*h];
      f = f*h;
    end
    
    function [f,df] = position_constraint_fun(obj,x,offset,phidot_only,data)
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
      nq = obj.plant.getNumPositions;
      nC = obj.nC;
      nU = obj.plant.getNumInputs();

      q = x(1:nq);
      v = x(nq+1:end);
      
      [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics(q,v);
      Hinv = inv(H);
      if nC > 0      
        [phi,J,dJ] = obj.plant.positionConstraints(q);
%         phi = phi(obj.active_inds,:);  
%         J = J(obj.active_inds,:);      
%         dJ = dJ(obj.active_inds,:);
        dJ = reshape(dJ,[],nq);
        Jdot = matGradMult(dJ,v);      
        vdot = Hinv*(B*u - C + J'*lambda);
        dtau = matGradMult(dB,u) - dC + [matGradMult(dJ,lambda,true), zeros(nq,nq)];

        dvdot = [zeros(nq,1),...
          -Hinv*matGradMult(dH(:,1:nq),vdot) + Hinv*dtau(:,1:nq),...
          +Hinv*dtau(:,1+nq:end), Hinv*B, Hinv*J'];

        [VqInv,dVqInv] = obj.plant.vToqdot(q);
        xdot = [VqInv*v;vdot];
        dxdot = [...
          zeros(nq,1), matGradMult(dVqInv, v), VqInv, zeros(nq,nU+nC);
          dvdot];   

        data.xdot = xdot;
        data.dxdot = dxdot;
        data.phi = phi;
        data.J = J;
        data.Jdot = Jdot;      
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
        data.Jdot = zeros(0,nq);
        
      end
    end
    
    function obj = setupVariables(obj,N)
      obj = setupVariables@DircolTrajectoryOptimization(obj,N);
      
      
      nActive = length(obj.active_inds);
      nC = obj.plant.num_position_constraints;
      obj.nC = nC;
      obj.active_inds = obj.options.active_inds;
      num_vars = N*nC + (N-1)*nC;

      obj.l_inds = reshape(obj.num_vars + (1:nC*N),nC,N);
      obj.lc_inds = reshape(obj.num_vars + nC*N + (1:nC*(N-1)),nC,N-1);
     
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
      [xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTrajFromZ@DircolTrajectoryOptimization(obj,z0);
      t_l = zeros(obj.N*2-1,1);
      lambda = zeros(size(obj.l_inds,1),obj.N*2 -1);
      
      lambda(:,1:2:end) = z(obj.l_inds);
      lambda(:,2:2:end) = z(obj.lc_inds);
      ltraj = PPTrajectory(foh(t_l,lambda));
    end
    
    function [xtraj,utraj,ltraj,z,F,info,infeasible_constraint_name] = solveTraj(obj,varargin)
      [xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTraj@DircolTrajectoryOptimization(obj,varargin{:});      
      
      ltraj = obj.reconstructForceTrajectory(z);
    end
    
    function ltraj = reconstructForceTrajectory(obj,z)
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
      nq = obj.plant.getNumPositions;
      nU = obj.plant.getNumInputs();

      q = x(1:nq);
      v = x(nq+1:end);
      
      [H,C,B] = obj.plant.manipulatorDynamics(q,v);
      [~,J] = obj.plant.positionConstraints(q);     
%       J = J(obj.active_inds,:);  
      
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