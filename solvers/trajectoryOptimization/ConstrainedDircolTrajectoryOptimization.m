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
  end
  
  methods
    function obj = ConstrainedDircolTrajectoryOptimization(plant,N,duration,options)
      if nargin < 4
        options = struct();
      end
      
      nC = plant.num_position_constraints;
      
      if ~isfield(options,'relative_constraints')
        options.relative_constraints = false(nC,1);
      elseif length(options.relative_constraints) == 1
        options.relative_constraints = repmat(options.relative_constraints,nC,1);
      end
      
      if ~isfield(options,'lambda_bound')
        options.lambda_bound = 100;
      end
      
      obj = obj@DircolTrajectoryOptimization(plant,N,duration,options);
      
      % add constraints on lambda
      for i=1:nC,
        A_ub = [.5*eye(N-1) zeros(N-1,1) -eye(N-1) options.lambda_bound*eye(N-1)];
        A_ub(:,2:N) = A_ub(:,2:N) + .5*eye(N-1);
        obj = obj.addConstraint(LinearConstraint(zeros(N-1,1),inf(N-1,1),A_ub),[obj.l_inds(i,:)';obj.lc_inds(i,:)';obj.h_inds]);
        
        
        A_lb = [.5*eye(N-1) zeros(N-1,1) -eye(N-1) -options.lambda_bound*eye(N-1)];
        A_lb(:,2:N) = A_lb(:,2:N) + .5*eye(N-1);
        obj = obj.addConstraint(LinearConstraint(-inf(N-1,1),zeros(N-1,1),A_lb),[obj.l_inds(i,:)';obj.lc_inds(i,:)';obj.h_inds]);
      end
    end
    
    function obj = addDynamicConstraints(obj)
      N = obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      nC = obj.nC;
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
      
      for i=1:obj.N-1,
        dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i);obj.u_inds(:,i+1);
          obj.l_inds(:,i); obj.l_inds(:,i+1); obj.lc_inds(:,i)};
        dyn_constraints{i} = cnstr;
        
        obj = obj.addConstraint(dyn_constraints{i}, dyn_inds{i},[shared_data_index+i;shared_data_index+i+1]);
      end

      pos_cnstr = FunctionHandleConstraint(zeros(2*nC,1),zeros(2*nC,1),nX+length(obj.cstrval_inds),@obj.position_constraint_fun);
      for i=1:obj.N,
        obj = obj.addConstraint(pos_cnstr, {obj.x_inds(:,i),obj.cstrval_inds},shared_data_index+i);
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
      
      % constrait is the difference between the two
      f = xdotcol - g;
      df = dxdotcol - dg;
    end
    
    function [f,df] = position_constraint_fun(obj,x,offset,data)
      nq = obj.plant.getNumPositions; 
      
      q = x(1:nq);
      qd = x(nq+1:end);
      
      phi = data.phi;
      J = data.J;
      Jdot = data.Jdot;
      
      if nargin > 3 || true
        noffset = length(offset);
        f = [phi;J*qd];
        f(obj.cstr_to_val_map) = f(obj.cstr_to_val_map) - offset;
          df = [J zeros(obj.nC,nq+noffset);
                Jdot J zeros(obj.nC,noffset)];
          df(obj.cstr_to_val_map,end-noffset+1:end) = -eye(noffset);
        else
          f = [phi;J*qd];
          df = [J zeros(obj.nC,nq);
                Jdot J];
      end
    end
    
    function data = dynamics_data(obj,x,u,lambda)
      nq = obj.plant.getNumPositions;
      nC = obj.nC;
      nU = obj.plant.getNumInputs();

      q = x(1:nq);
      v = x(nq+1:end);
      
      [H,C,B,dH,dC,dB] = obj.plant.manipulatorDynamics(q,v);
      
      n_con = obj.nC/2;
      [phi,J,dJ] = obj.plant.positionConstraints(q);     
      dJ = reshape(dJ,[],nq);
      
     
      Jdot = matGradMult(dJ,v);      
      
      Hinv = inv(H);
      
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
    end
    
    function obj = setupVariables(obj,N)
      obj = setupVariables@DircolTrajectoryOptimization(obj,N);
      
      
      nC = obj.plant.num_position_constraints;
      obj.nC = nC;
      num_vars = N*nC + (N-1)*nC;

      obj.l_inds = reshape(obj.num_vars + (1:nC*N),nC,N);
      obj.lc_inds = reshape(obj.num_vars + nC*N + (1:nC*(N-1)),nC,N-1);
     
      obj = obj.addDecisionVariable(num_vars);
      
      if any(obj.options.relative_constraints)
        obj.cstr_to_val_map = find(obj.options.relative_constraints);
      else
        obj.cstr_to_val_map = zeros(0,1);
      end
      num_rel_constraints = length(obj.cstr_to_val_map);
      obj.cstrval_inds = (1:num_rel_constraints)' + obj.num_vars;
      
      obj = obj.addDecisionVariable(num_rel_constraints);
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
      [phi,J] = obj.plant.positionConstraints(q);     
      
      vdot = H\(B*u - C + J'*lambda);
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