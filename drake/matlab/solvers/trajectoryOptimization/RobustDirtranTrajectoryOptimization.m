 classdef RobustDirtranTrajectoryOptimization < DirectTrajectoryOptimization
    %  For forward euler integratino:
    %    dynamics constraints are: x(k+1) = x(k) + h(k)*f(x(k),u(k))
    %    integrated cost is sum of g(h(k),x(k),u(k))
    %  For midpoint integration:
    %    dynamics constraints are: x(k+1) = x(k) + h(k)*f(.5*x(k)+.5*x(k+1),u(k))
    %    integrated cost is sum of g(h(k),.5*x(k)+.5*x(k+1),u(k))
    
  properties (Constant)
    FORWARD_EULER = 1;
    MIDPOINT = 3;  % DEFAULT
  end
  
  properties
    nX
    nU
    nW
    
    D % Disturbance ellipsoid matrix w'*D^-1*w <= 1
    E0 % Initial disturbed state at t=0
    
    Q % LQR state cost matrix
    R % LQR input cost matrix
    Qf% LQR terminal cost matrix
    
    Qr %Robust cost matrix
    Rr %Robust cost matrix
    Qrf %Robust cost matrix
    
    Pc %Projection onto constrained subspace of state vector
    
    %Stuff to cache so we don't have to recompute LQR controller
    z_handle
    K_handle
    A_handle
    B_handle
    G_handle
    dK_handle
    dA_handle
    dB_handle
    dG_handle
    E_handle
    dE_handle
    
    %Stuff for robustifying state constraints
    constr_xinds
    dUdE
    dxcdv
    dxc
    
  end
  
  methods
    function obj = RobustDirtranTrajectoryOptimization(plant,N,D,E0,Q,R,Qf,duration,options)
      
      if nargin < 9
        options = struct();
      end
      if ~isfield(options,'integration_method')
        options.integration_method = RobustDirtranTrajectoryOptimization.MIDPOINT;
      end
      if isscalar(duration)
          duration=[duration,duration];
      end

      obj = obj@DirectTrajectoryOptimization(plant,N,duration,options);
      
      obj.nX = plant.getNumStates;
      obj.nU = plant.getNumInputs;
      obj.nW = plant.getNumDisturbances;
      obj.D = D;
      obj.E0 = E0;
      obj.Q = Q;
      obj.R = R;
      obj.Qf = Qf;
      
      obj.z_handle = SharedDataHandle(0);
      obj.K_handle = SharedDataHandle(0);
      obj.A_handle = SharedDataHandle(0);
      obj.B_handle = SharedDataHandle(0);
      obj.G_handle = SharedDataHandle(0);
      obj.dK_handle = SharedDataHandle(0);
      obj.dA_handle = SharedDataHandle(0);
      obj.dB_handle = SharedDataHandle(0);
      obj.dG_handle = SharedDataHandle(0);
      obj.E_handle = SharedDataHandle(0);
      obj.dE_handle = SharedDataHandle(0);
      
    end
    
    function obj = setupVariables(obj, N)
      % Assumes that there are N-1 time steps
      % N corresponding state variables
      % and N-1 corresponding input variables
      %
      % Generates num_vars total number of decision variables
      %   h_inds (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
      %   x_inds N x n indices for state
      %   u_inds (N-1) x m indices for input
      %
      % @param N number of knot points
      nH = N-1;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();

      num_vars = nH + N*nX + (N-1)*nU;
      obj.h_inds = (1:nH)';
      obj.x_inds = reshape(nH + (1:nX*N),nX,N);
      obj.u_inds = reshape(nH + nX*N + (1:nU*(N-1)),nU,N-1);

      obj.N = N;
      x_names = cell(num_vars,1);
      for i = 1:(N-1)
        x_names{i} = sprintf('h[%d]',i);
        for j = 1:nX
          x_names{nH+(i-1)*nX+j}=sprintf('x%d[%d]',j,i);
        end
        for j = 1:nU
          x_names{nH+nX*N+(i-1)*nU+j} = sprintf('u%d[%d]',j,i);
        end
      end
      for j = 1:nX
          x_names{nH+(N-1)*nX+j}=sprintf('x%d[%d]',j,N);
      end

      obj = obj.addDecisionVariable(num_vars,x_names);
      
      % Ensure that all h values are non-negative
      obj = obj.addConstraint(BoundingBoxConstraint(zeros(N-1,1),inf(N-1,1)),obj.h_inds);
      
      % create constraints for dynamics and add them
      obj = obj.addDynamicConstraints();
      
      % add control inputs as bounding box constraints
      if any(~isinf(obj.plant.umin)) || any(~isinf(obj.plant.umax))
          control_limit = BoundingBoxConstraint(repmat(obj.plant.umin,N-1,1),repmat(obj.plant.umax,N-1,1));
          obj = obj.addConstraint(control_limit,obj.u_inds(:));
      end
      
      obj.z_handle.data = randn((N-1)*(1+nX+nU)+nX,1);
      
    end
    
    function z0 = getInitialVars(obj,t_init,traj_init)
        % evaluates the initial trajectories at the sampled times and
        % constructs the nominal z0.
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
            z0(obj.u_inds) = traj_init.u.eval(t_init(1:end-1));
        else
            z0(obj.u_inds) = 0.01*randn(nU,obj.N-1);
        end
        
        if isfield(traj_init,'x')
            z0(obj.x_inds) = traj_init.x.eval(t_init);
        else
            if nU>0
                if ~isfield(traj_init,'u')
                    traj_init.u = setOutputFrame(PPTrajectory(foh(t_init,reshape(z0(obj.u_inds),nU,obj.N-1))),getInputFrame(obj.plant));
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
    
    function obj = addDynamicConstraints(obj)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      N = obj.N;
      
      constraints = cell(N-1,1);
      dyn_inds = cell(N-1,1);
      
      switch obj.options.integration_method
        case RobustDirtranTrajectoryOptimization.FORWARD_EULER
          n_vars = 2*nX + nU + 1;
          cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.forward_constraint_fun);
        case RobustDirtranTrajectoryOptimization.MIDPOINT
          n_vars = 2*nX + nU + 1;
          cnstr = FunctionHandleConstraint(zeros(nX,1),zeros(nX,1),n_vars,@obj.midpoint_constraint_fun);
        otherwise
          error('Drake:RobustDirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
      end
      
      for i=1:obj.N-1,
        switch obj.options.integration_method
          case RobustDirtranTrajectoryOptimization.FORWARD_EULER
            dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
          case RobustDirtranTrajectoryOptimization.MIDPOINT
            dyn_inds{i} = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
          otherwise
            error('Drake:RobustDirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
        end
        cnstr = cnstr.setName(sprintf('dynamics_constr_%d_',i));
        constraints{i} = cnstr;
        
        obj = obj.addConstraint(constraints{i}, dyn_inds{i});
      end
    end
    
    function obj = addRunningCost(obj,running_cost_function,grad_level)
      % Adds an integrated cost to all time steps, which is
      % numerical implementation specific (thus abstract)
      % this cost is assumed to be time-invariant
      % @param running_cost_function a function handle
      %  of the form running_cost_function(dt,x,u)
      
      if nargin < 3
        grad_level = -1;
      end
      
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      
      for i=1:obj.N-1,
        switch obj.options.integration_method
          case RobustDirtranTrajectoryOptimization.FORWARD_EULER
            running_cost = FunctionHandleObjective(1+nX+nU, running_cost_function,grad_level);
            inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.u_inds(:,i)};
          case RobustDirtranTrajectoryOptimization.MIDPOINT
            running_cost = FunctionHandleObjective(1+2*nX+nU,...
              @(h,x0,x1,u0) obj.midpoint_running_fun(running_cost_function,h,x0,x1,u0),grad_level);
            inds_i = {obj.h_inds(i);obj.x_inds(:,i);obj.x_inds(:,i+1);obj.u_inds(:,i)};
          otherwise
            error('Drake:RobustDirtranTrajectoryOptimization:InvalidArgument','Unknown integration method');
        end
        
        obj = obj.addCost(running_cost,inds_i);
      end
    end
    
    function obj = addRobustCost(obj,Qr,Rr,Qrf)
        nX = obj.nX;
        nU = obj.nU;
        N = obj.N;
        
        obj.Qr = Qr;
        obj.Rr = Rr;
        obj.Qrf = Qrf;
        
        dim = N-1 + N*nX + (N-1)*nU;
        cost = FunctionHandleObjective(dim,@obj.robust_cost,1);
        cost.grad_method = 'user';
        obj = obj.addCost(cost, {reshape([obj.h_inds'; obj.x_inds(:,1:end-1); obj.u_inds],[],1); obj.x_inds(:,end)});
    end
    
    function obj = addRobustInputConstraint(obj)
        nX = obj.nX;
        nU = obj.nU;
        nW = obj.nW;
        N = obj.N;
        
        lb = repmat(obj.plant.umin,2*nU*(N-2),1);
        ub = repmat(obj.plant.umax,2*nU*(N-2),1);
        constraint = FunctionHandleConstraint(lb,ub,N-1+N*nX+(N-1)*nU,@obj.robust_input_constraint,1);
        constraint.grad_method = 'user';
        obj = obj.addConstraint(constraint, {reshape([obj.h_inds'; obj.x_inds(:,1:end-1); obj.u_inds],[],1); obj.x_inds(:,end)});
    end
    
    function obj = addRobustStateConstraint(obj,single_constr,times,x_ind)
        nX = obj.nX;
        nU = obj.nU;
        nW = obj.nW;
        N = obj.N;
        nXc = length(x_ind);
        nCout = single_constr.num_cnstr;
        nC = 2*length(times)*nXc;
        
        %Projection matrix onto constrained subspace
        obj.Pc = sparse((1:nXc)', x_ind, ones(nXc,1), nXc, nX, nXc);
        obj.dUdE = kron(obj.Pc, obj.Pc);
        
        lb = zeros(nC*nCout, 1);
        ub = zeros(nC*nCout, 1);
        for k = 1:nC
            lb((k-1)*nCout + (1:nCout)) = single_constr.lb;
            ub((k-1)*nCout + (1:nCout)) = single_constr.ub;
        end
        
        %these only need to be computed once
        obj.constr_xinds = kron(1+(1:N-2)*(1+nX+nU), ones(length(x_ind),1)) + kron(ones(1,N-2), x_ind(:));
        xcinds = [kron(obj.constr_xinds, ones(1,nXc)), kron(obj.constr_xinds, ones(1,nXc))];
        obj.dxc = sparse((1:(nC*nXc))', xcinds(:), ones(length(xcinds(:)),1), nC*nXc, N-1+N*nX+(N-1)*nU);
        obj.dxcdv = sparse(1:(nC*nXc), [1:(nC*nXc/2) 1:(nC*nXc/2)], [ones(nC*nXc/2,1); -ones(nC*nXc/2,1)], nC*nXc, nC*nXc/2);
        
        constraint = FunctionHandleConstraint(lb,ub,N-1+N*nX+(N-1)*nU,@(y,xf)obj.robust_state_constraint(y,xf,single_constr,times,x_ind),1);
        constraint.grad_method = 'user';
        constraint.grad_level = 1;
        obj = obj.addConstraint(constraint, {reshape([obj.h_inds'; obj.x_inds(:,1:end-1); obj.u_inds],[],1); obj.x_inds(:,end)});
    end
    
    function [f,df] = forward_constraint_fun(obj,h,x0,x1,u)
      nX = obj.plant.getNumStates();
      [xdot,dxdot] = obj.plant.dynamics(0,x0,u);
      f = x1 - x0 - h*xdot;
      df = [-xdot (-eye(nX) - h*dxdot(:,2:1+nX)) eye(nX) -h*dxdot(:,nX+2:end)];
    end
    
    function [f,df] = midpoint_running_fun(obj,running_handle,h,x0,x1,u0)
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      [f,dg] = running_handle(h,.5*(x0+x1),u0);
      
      df = [dg(:,1) .5*dg(:,2:1+nX) .5*dg(:,2:1+nX) dg(:,2+nX:1+nX+nU)];
    end
    
    function [f,df] = midpoint_constraint_fun(obj,h,x0,x1,u0)
      nX = obj.plant.getNumStates();
      [xdot,dxdot] = obj.plant.dynamics(0,.5*(x0+x1),u0);
      f = x1 - x0 - h*xdot;
      df = [-xdot (-eye(nX) - .5*h*dxdot(:,2:1+nX)) (eye(nX)- .5*h*dxdot(:,2:1+nX)) -h*dxdot(:,nX+2:end)];
    end
    
    function [c, dc] = robust_cost(obj,y,xf)
        nX = obj.nX;
        nU = obj.nU;
        nW = obj.nW;
        N = obj.N;
        
        [K,A,B,G,E,dK,dA,dB,dG,dE] = deltaLQR(obj,y,xf);
        
        c = 0;
        dc = zeros(1,(N-1)*(1+nX+nU)+nX);
        for k = 1:(N-1)
            c = c + trace((obj.Qr + K(:,:,k)'*obj.Rr*K(:,:,k))*E(:,:,k));
            
            dcdE = vec((obj.Qr + K(:,:,k)'*obj.Rr*K(:,:,k)))';
            dcdK = 2*vec(obj.Rr*K(:,:,k)*E(:,:,k))';
            
            dc = dc + dcdE*dE(:,:,k) + dcdK*dK(:,:,k);
        end
        c = c + trace(obj.Qrf*E(:,:,N));
        dcdE = vec(obj.Qrf)';
        dc = dc + dcdE*dE(:,:,N);
    end

    function [c, dc] = robust_input_constraint(obj,y,xf)
        nX = obj.nX;
        nU = obj.nU;
        nW = obj.nW;
        N = obj.N;
        
        [K,~,~,~,E,dK,~,~,~,dE] = deltaLQR(obj,y,xf);
        
        v = zeros((N-2)*nU*nU,1);
        dv = zeros((N-2)*nU*nU,(N-1)*(1+nX+nU)+nX);
        
        for k = 2:(N-1)
            U = K(:,:,k)*E(:,:,k)*K(:,:,k)';
            Us = obj.fastsqrt(U);
            v((k-2)*(nU*nU)+(1:nU*nU)) = vec(Us);
            
            dvdU = inv(kron(eye(nU),Us) + kron(Us,eye(nU)));
            dUdK = kron(K(:,:,k)*E(:,:,k), eye(nU)) + kron(eye(nU), K(:,:,k)*E(:,:,k))*comm(nU,nX);
            dUdE = kron(K(:,:,k), K(:,:,k));
            dvdK = dvdU*dUdK;
            dvdE = dvdU*dUdE;
            
            dv((k-2)*(nU*nU)+(1:nU*nU),:) = dvdK*dK(:,:,k) + dvdE*dE(:,:,k);
            
        end
        
        uinds = kron(1+nX+(1:N-2)*(1+nX+nU), ones(nU,1)) + kron(ones(1,N-2), (1:nU)');
        u = y(uinds);
        uc = vec(kron(ones(nU,1), u));
        c = [uc+v(:); uc-v(:)];
        
        du = sparse(1:((N-2)*nU*nU), vec(kron(ones(nU,1), uinds)), ones((N-2)*nU*nU,1),(N-2)*nU*nU,(N-1)*(1+nX+nU)+nX);
        dc = [du+dv; du-dv];
    end
    
    function [c, dc] = robust_state_constraint(obj,y,xf,constr_fun,times,x_ind)
        nX = obj.nX;
        nU = obj.nU;
        nW = obj.nW;
        N = obj.N;
        
        nXc = length(x_ind); %number of constrained components
        nC = 2*length(times)*nXc; %number of constraint vectors
        nCout = constr_fun.num_cnstr; %number of components in each constraint vector
        
        Pc = obj.Pc; %projection onto constrained subspace
        dUdE = obj.dUdE;
        
        [~,~,~,~,E,~,~,~,~,dE] = deltaLQR(obj,y,xf);
        
        v = zeros(nXc,(N-2)*nXc);
        dv = zeros((N-2)*nXc*nXc,(N-1)*(1+nX+nU)+nX);
        for k = 2:(N-1)
            U = Pc*E(:,:,k)*Pc';
            Us = obj.fastsqrt(U);
            v(:,(k-2)*nXc + (1:nXc)') = Us;
            
            dvdU_inv = kron(eye(nXc),Us) + kron(Us,eye(nXc));
            dvdE = dvdU_inv\dUdE;
            
            dv((k-2)*(nXc*nXc)+(1:(nXc*nXc))',:) = dvdE*dE(:,:,k);
        end

        x = kron(y(obj.constr_xinds), ones(1,nXc));
        xc = [x+v x-v];
        
        %Evaluate constraint function
        c = zeros(nC,1);
        dcdxc = sparse([],[],[],nC,nC*nXc,nC*nXc);
        for k = 1:nC
            [c((k-1)*nCout+(1:nCout)), dcdxc((k-1)*nCout+(1:nCout),(k-1)*nXc+(1:nXc))] = constr_fun.eval(xc(:,k));
        end
        
        dc = dcdxc*obj.dxc + dcdxc*obj.dxcdv*dv;
    end
    
    function [K,A,B,G,E,dK,dA,dB,dG,dE] = deltaLQR(obj,y,xf)
        nX = obj.nX;
        nU = obj.nU;
        nW = obj.nW;
        N = obj.N;
        
        %check to see if we actually need to do anything
        if any([y; xf] ~= obj.z_handle.data)
            
            %Get dynamics derivatives along trajectory
            A = zeros(nX,nX,N-1);
            B = zeros(nX,nU,N-1);
            G = zeros(nX,nW,N-1);
            switch obj.options.integration_method
                case RobustDirtranTrajectoryOptimization.FORWARD_EULER
                    dA = zeros(nX*nX,1+nX+nU,N-1);
                    dB = zeros(nX*nU,1+nX+nU,N-1);
                    dG = zeros(nX*nW,1+nX+nU,N-1);
                    for k = 1:(N-1)
                        [~,dx,d2x] = obj.robust_dynamics(y((k-1)*(1+nX+nU)+1),y((k-1)*(1+nX+nU)+1+(1:nX)),y((k-1)*(1+nX+nU)+1+nX+(1:nU)),zeros(nW,1));
                        A(:,:,k) = dx(:,1+(1:nX));
                        B(:,:,k) = dx(:,1+nX+(1:nU));
                        G(:,:,k) = dx(:,1+nX+nU+(1:nW));
                        dvec = reshape(d2x,nX*(1+nX+nU+nW),1+nX+nU+nW);
                        dA(:,:,k) = dvec(nX+(1:nX*nX),1:(1+nX+nU));
                        dB(:,:,k) = dvec((1+nX)*nX+(1:nX*nU),1:(1+nX+nU));
                        dG(:,:,k) = dvec((1+nX+nU)*nX+(1:nX*nW),1:(1+nX+nU));
                    end
                    
                    %Solve Riccati Equation
                    P = obj.Qf;
                    dP = zeros(nX*nX,(N-1)*(1+nX+nU)+nX);
                    K = zeros(nU,nX,N-1);
                    dK = zeros(nU*nX,(N-1)*(1+nX+nU)+nX,N-1);
                    for k = (N-1):-1:1
                        K(:,:,k) = (B(:,:,k).'*P*B(:,:,k)+obj.R)\(B(:,:,k).'*P*A(:,:,k));
                        dKdA = kron(eye(nX),(B(:,:,k)'*P*B(:,:,k)+obj.R)\B(:,:,k)'*P);
                        dKdB = kron(A(:,:,k)'*P, inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*comm(nX,nU) - kron(A(:,:,k)'*P*B(:,:,k), eye(nU))*kron(inv(B(:,:,k)'*P*B(:,:,k)+obj.R)', inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*(kron(eye(nU), B(:,:,k)'*P) + kron(B(:,:,k)'*P, eye(nU))*comm(nX,nU));
                        dKdP = kron(A(:,:,k)', (B(:,:,k)'*P*B(:,:,k)+obj.R)\B(:,:,k)') - kron(A(:,:,k)'*P*B(:,:,k), eye(nU))*kron(inv(B(:,:,k)'*P*B(:,:,k)+obj.R)', inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*kron(B(:,:,k)', B(:,:,k)');
                        dK(:,:,k) = dKdP*dP;
                        dK(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU)),k) = dK(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU)),k) + dKdA*dA(:,:,k) + dKdB*dB(:,:,k);
                        
                        dPdA = kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P) + kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*comm(nX,nX);
                        dPdB = -kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P)*kron(K(:,:,k)', eye(nX)) - kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*kron(eye(nX), K(:,:,k)')*comm(nX,nU);
                        dPdK = kron(eye(nX), K(:,:,k)'*obj.R) + kron(K(:,:,k)'*obj.R, eye(nX))*comm(nU,nX) - kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P)*kron(eye(nX), B(:,:,k)) - kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*kron(B(:,:,k), eye(nX))*comm(nU,nX);
                        dPdP = kron((A(:,:,k)-B(:,:,k)*K(:,:,k))', (A(:,:,k)-B(:,:,k)*K(:,:,k))');
                        
                        P = obj.Q + K(:,:,k).'*obj.R*K(:,:,k) + (A(:,:,k) - B(:,:,k)*K(:,:,k)).'*P*(A(:,:,k) - B(:,:,k)*K(:,:,k));
                        dP = dPdP*dP + dPdK*dK(:,:,k);
                        dP(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU))) = dP(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU))) + dPdA*dA(:,:,k) + dPdB*dB(:,:,k);
                    end
                case RobustDirtranTrajectoryOptimization.MIDPOINT
                    dA = zeros(nX*nX,2*(1+nX+nU),N-1);
                    dB = zeros(nX*nU,2*(1+nX+nU),N-1);
                    dG = zeros(nX*nW,2*(1+nX+nU),N-1);
                    for k = 1:(N-2)
                        [~,dx,d2x] = obj.robust_dynamics(y((k-1)*(1+nX+nU)+1),.5*(y((k-1)*(1+nX+nU)+1+(1:nX))+y((k)*(1+nX+nU)+1+(1:nX))),y((k-1)*(1+nX+nU)+1+nX+(1:nU)),zeros(nW,1));
                        A(:,:,k) = dx(:,1+(1:nX));
                        B(:,:,k) = dx(:,1+nX+(1:nU));
                        G(:,:,k) = dx(:,1+nX+nU+(1:nW));
                        dvec = reshape(d2x,nX*(1+nX+nU+nW),1+nX+nU+nW);
                        dA(:,:,k) = [dvec(nX+(1:nX*nX),1), .5*dvec(nX+(1:nX*nX),1+(1:nX)), dvec(nX+(1:nX*nX),1+nX+(1:nU)), zeros(nX*nX,1), .5*dvec(nX+(1:nX*nX),1+(1:nX)), zeros(nX*nX,nU)];
                        dB(:,:,k) = [dvec((1+nX)*nX+(1:nX*nU),1), .5*dvec((1+nX)*nX+(1:nX*nU),1+(1:nX)), dvec((1+nX)*nX+(1:nX*nU),1+nX+(1:nU)), zeros(nX*nU,1), .5*dvec((1+nX)*nX+(1:nX*nU),1+(1:nX)), zeros(nX*nU,nU)];
                        dG(:,:,k) = [dvec((1+nX+nU)*nX+(1:nX*nW),1), .5*dvec((1+nX+nU)*nX+(1:nX*nW),1+(1:nX)), dvec((1+nX+nU)*nX+(1:nX*nW),1+nX+(1:nU)), zeros(nX*nW,1), .5*dvec((1+nX+nU)*nX+(1:nX*nW),1+(1:nX)), zeros(nX*nW,nU)];
                    end
                    k = N-1;
                    [~,dx,d2x] = obj.robust_dynamics(y((k-1)*(1+nX+nU)+1),.5*(y((k-1)*(1+nX+nU)+1+(1:nX))+xf),y((k-1)*(1+nX+nU)+1+nX+(1:nU)),zeros(nW,1));
                    A(:,:,k) = dx(:,1+(1:nX));
                    B(:,:,k) = dx(:,1+nX+(1:nU));
                    G(:,:,k) = dx(:,1+nX+nU+(1:nW));
                    dvec = reshape(d2x,nX*(1+nX+nU+nW),1+nX+nU+nW);
                    dA(:,:,k) = [dvec(nX+(1:nX*nX),1), .5*dvec(nX+(1:nX*nX),1+(1:nX)), dvec(nX+(1:nX*nX),1+nX+(1:nU)), zeros(nX*nX,1), .5*dvec(nX+(1:nX*nX),1+(1:nX)), zeros(nX*nX,nU)];
                    dB(:,:,k) = [dvec((1+nX)*nX+(1:nX*nU),1), .5*dvec((1+nX)*nX+(1:nX*nU),1+(1:nX)), dvec((1+nX)*nX+(1:nX*nU),1+nX+(1:nU)), zeros(nX*nU,1), .5*dvec((1+nX)*nX+(1:nX*nU),1+(1:nX)), zeros(nX*nU,nU)];
                    dG(:,:,k) = [dvec((1+nX+nU)*nX+(1:nX*nW),1), .5*dvec((1+nX+nU)*nX+(1:nX*nW),1+(1:nX)), dvec((1+nX+nU)*nX+(1:nX*nW),1+nX+(1:nU)), zeros(nX*nW,1), .5*dvec((1+nX+nU)*nX+(1:nX*nW),1+(1:nX)), zeros(nX*nW,nU)];
                    
                    %Solve Riccati Equation
                    P = obj.Qf;
                    dP = zeros(nX*nX,(N-1)*(1+nX+nU)+nX);
                    K = zeros(nU,nX,N-1);
                    dK = zeros(nU*nX,(N-1)*(1+nX+nU)+nX,N-1);
                    
                    k = N-1;
                    K(:,:,k) = (B(:,:,k).'*P*B(:,:,k)+obj.R)\(B(:,:,k).'*P*A(:,:,k));
                    dKdA = kron(eye(nX),(B(:,:,k)'*P*B(:,:,k)+obj.R)\B(:,:,k)'*P);
                    dKdB = kron(A(:,:,k)'*P, inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*comm(nX,nU) - kron(A(:,:,k)'*P*B(:,:,k), eye(nU))*kron(inv(B(:,:,k)'*P*B(:,:,k)+obj.R)', inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*(kron(eye(nU), B(:,:,k)'*P) + kron(B(:,:,k)'*P, eye(nU))*comm(nX,nU));
                    dKdP = kron(A(:,:,k)', (B(:,:,k)'*P*B(:,:,k)+obj.R)\B(:,:,k)') - kron(A(:,:,k)'*P*B(:,:,k), eye(nU))*kron(inv(B(:,:,k)'*P*B(:,:,k)+obj.R)', inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*kron(B(:,:,k)', B(:,:,k)');
                    dK(:,:,k) = dKdP*dP;
                    dK(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU)),k) = dKdA*dA(:,1:(1+nX+nU),k) + dKdB*dB(:,1:(1+nX+nU),k);
                    dK(:,k*(1+nX+nU)+(1:nX),k) = dKdA*dA(:,(1+nX+nU+1)+(1:nX),k) + dKdB*dB(:,(1+nX+nU+1)+(1:nX),k);
                    dPdA = kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P) + kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*comm(nX,nX);
                    dPdB = -kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P)*kron(K(:,:,k)', eye(nX)) - kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*kron(eye(nX), K(:,:,k)')*comm(nX,nU);
                    dPdK = kron(eye(nX), K(:,:,k)'*obj.R) + kron(K(:,:,k)'*obj.R, eye(nX))*comm(nU,nX) - kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P)*kron(eye(nX), B(:,:,k)) - kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*kron(B(:,:,k), eye(nX))*comm(nU,nX);
                    dPdP = kron((A(:,:,k)-B(:,:,k)*K(:,:,k))', (A(:,:,k)-B(:,:,k)*K(:,:,k))');
                    P = obj.Q + K(:,:,k).'*obj.R*K(:,:,k) + (A(:,:,k) - B(:,:,k)*K(:,:,k)).'*P*(A(:,:,k) - B(:,:,k)*K(:,:,k));
                    dP = dPdP*dP + dPdK*dK(:,:,k);
                    dP(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU))) = dP(:,(k-1)*(1+nX+nU)+(1:(1+nX+nU))) + dPdA*dA(:,1:(1+nX+nU),k) + dPdB*dB(:,1:(1+nX+nU),k);
                    dP(:,k*(1+nX+nU)+(1:nX)) = dP(:,k*(1+nX+nU)+(1:nX))+ dPdA*dA(:,(1+nX+nU+1)+(1:nX),k) + dPdB*dB(:,(1+nX+nU+1)+(1:nX),k);
                    for k = (N-2):-1:1
                        K(:,:,k) = (B(:,:,k).'*P*B(:,:,k)+obj.R)\(B(:,:,k).'*P*A(:,:,k));
                        dKdA = kron(eye(nX),(B(:,:,k)'*P*B(:,:,k)+obj.R)\B(:,:,k)'*P);
                        dKdB = kron(A(:,:,k)'*P, inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*comm(nX,nU) - kron(A(:,:,k)'*P*B(:,:,k), eye(nU))*kron(inv(B(:,:,k)'*P*B(:,:,k)+obj.R)', inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*(kron(eye(nU), B(:,:,k)'*P) + kron(B(:,:,k)'*P, eye(nU))*comm(nX,nU));
                        dKdP = kron(A(:,:,k)', (B(:,:,k)'*P*B(:,:,k)+obj.R)\B(:,:,k)') - kron(A(:,:,k)'*P*B(:,:,k), eye(nU))*kron(inv(B(:,:,k)'*P*B(:,:,k)+obj.R)', inv(B(:,:,k)'*P*B(:,:,k)+obj.R))*kron(B(:,:,k)', B(:,:,k)');
                        dK(:,:,k) = dKdP*dP;
                        dK(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU)),k) = dK(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU)),k) + dKdA*dA(:,:,k) + dKdB*dB(:,:,k);
                        
                        dPdA = kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P) + kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*comm(nX,nX);
                        dPdB = -kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P)*kron(K(:,:,k)', eye(nX)) - kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*kron(eye(nX), K(:,:,k)')*comm(nX,nU);
                        dPdK = kron(eye(nX), K(:,:,k)'*obj.R) + kron(K(:,:,k)'*obj.R, eye(nX))*comm(nU,nX) - kron(eye(nX), (A(:,:,k)-B(:,:,k)*K(:,:,k))'*P)*kron(eye(nX), B(:,:,k)) - kron((A(:,:,k)-B(:,:,k)*K(:,:,k))'*P, eye(nX))*kron(B(:,:,k), eye(nX))*comm(nU,nX);
                        dPdP = kron((A(:,:,k)-B(:,:,k)*K(:,:,k))', (A(:,:,k)-B(:,:,k)*K(:,:,k))');
                        
                        P = obj.Q + K(:,:,k).'*obj.R*K(:,:,k) + (A(:,:,k) - B(:,:,k)*K(:,:,k)).'*P*(A(:,:,k) - B(:,:,k)*K(:,:,k));
                        dP = dPdP*dP + dPdK*dK(:,:,k);
                        dP(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU))) = dP(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU))) + dPdA*dA(:,:,k) + dPdB*dB(:,:,k);
                    end
            end
            
            E = zeros(nX, nX, N);
            E(:,:,1) = obj.E0;
            dE = zeros(nX*nX, (N-1)*(1+nX+nU)+nX, N);
            H = zeros(nX, nW);
            dH = zeros(nX*nW, (N-1)*(1+nX+nU)+nX);
            switch obj.options.integration_method
                case RobustDirtranTrajectoryOptimization.FORWARD_EULER
                    for k = 1:(N-1)
                        dEdA = kron(eye(nX), A(:,:,k)*E(:,:,k))*comm(nX,nX) + kron(A(:,:,k)*E(:,:,k), eye(nX)) - kron(B(:,:,k)*K(:,:,k)*E(:,:,k), eye(nX)) - kron(eye(nX), B(:,:,k)*K(:,:,k)*E(:,:,k))*comm(nX,nX) + kron(G(:,:,k)*H',eye(nX)) + kron(eye(nX),G(:,:,k)*H')*comm(nX,nX);
                        dEdB = -kron(eye(nX), A(:,:,k)*E(:,:,k)*K(:,:,k)')*comm(nX,nU) - kron(A(:,:,k)*E(:,:,k)*K(:,:,k)', eye(nX)) + kron(eye(nX), B(:,:,k)*K(:,:,k)*E(:,:,k)*K(:,:,k)')*comm(nX,nU) + kron(B(:,:,k)*K(:,:,k)*E(:,:,k)*K(:,:,k)', eye(nX)) - kron(G(:,:,k)*H'*K(:,:,k)',eye(nX)) - kron(eye(nX),G(:,:,k)*H'*K(:,:,k)')*comm(nX,nU);
                        dEdG = kron(eye(nX),(A(:,:,k)-B(:,:,k)*K(:,:,k))*H)*comm(nX,nW) + kron((A(:,:,k)-B(:,:,k)*K(:,:,k))*H,eye(nX)) + kron(eye(nX), G(:,:,k)*obj.D)*comm(nX,nW) + kron(G(:,:,k)*obj.D, eye(nX));
                        dEdK = -kron(B(:,:,k), A(:,:,k)*E(:,:,k))*comm(nU,nX) - kron(A(:,:,k)*E(:,:,k), B(:,:,k)) + kron(B(:,:,k)*K(:,:,k)*E(:,:,k), B(:,:,k)) + kron(B(:,:,k),B(:,:,k)*K(:,:,k)*E(:,:,k))*comm(nU,nX) - kron(G(:,:,k)*H',B(:,:,k)) - kron(B(:,:,k),G(:,:,k)*H')*comm(nU,nX);
                        dEdH = kron(G(:,:,k), A(:,:,k)-B(:,:,k)*K(:,:,k)) + kron(A(:,:,k)-B(:,:,k)*K(:,:,k), G(:,:,k))*comm(nX,nW);
                        dEdE = kron(A(:,:,k)-B(:,:,k)*K(:,:,k), A(:,:,k)-B(:,:,k)*K(:,:,k));
                        
                        dHdA = kron(H', eye(nX));
                        dHdB = -kron((K(:,:,k)*H)', eye(nX));
                        dHdG = kron(obj.D, eye(nX));
                        dHdK = -kron(H', B(:,:,k));
                        dHdH = kron(eye(nW), (A(:,:,k)-B(:,:,k)*K(:,:,k)));
                        
                        E(:,:,k+1) = (A(:,:,k)-B(:,:,k)*K(:,:,k))*E(:,:,k)*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + (A(:,:,k)-B(:,:,k)*K(:,:,k))*H*G(:,:,k)' + G(:,:,k)*H'*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + G(:,:,k)*obj.D*G(:,:,k)';
                        H = (A(:,:,k)-B(:,:,k)*K(:,:,k))*H + G(:,:,k)*obj.D;
                        
                        dE(:,:,k+1) = dEdE*dE(:,:,k) + dEdH*dH + dEdK*dK(:,:,k);
                        dE(:,(k-1)*(1+nX+nU)+(1:1+nX+nU),k+1) = dE(:,(k-1)*(1+nX+nU)+(1:1+nX+nU),k+1) + dEdA*dA(:,:,k) + dEdB*dB(:,:,k) + dEdG*dG(:,:,k);
                        
                        dH = dHdH*dH + dHdK*dK(:,:,k);
                        dH(:,(k-1)*(1+nX+nU)+(1:1+nX+nU)) = dH(:,(k-1)*(1+nX+nU)+(1:1+nX+nU)) + dHdA*dA(:,:,k) + dHdB*dB(:,:,k) + dHdG*dG(:,:,k);
                    end
                case RobustDirtranTrajectoryOptimization.MIDPOINT
                    for k = 1:(N-2)
                        
                        dEdA = kron(eye(nX), A(:,:,k)*E(:,:,k))*comm(nX,nX) + kron(A(:,:,k)*E(:,:,k), eye(nX)) - kron(B(:,:,k)*K(:,:,k)*E(:,:,k), eye(nX)) - kron(eye(nX), B(:,:,k)*K(:,:,k)*E(:,:,k))*comm(nX,nX) + kron(G(:,:,k)*H',eye(nX)) + kron(eye(nX),G(:,:,k)*H')*comm(nX,nX);
                        dEdB = -kron(eye(nX), A(:,:,k)*E(:,:,k)*K(:,:,k)')*comm(nX,nU) - kron(A(:,:,k)*E(:,:,k)*K(:,:,k)', eye(nX)) + kron(eye(nX), B(:,:,k)*K(:,:,k)*E(:,:,k)*K(:,:,k)')*comm(nX,nU) + kron(B(:,:,k)*K(:,:,k)*E(:,:,k)*K(:,:,k)', eye(nX)) - kron(G(:,:,k)*H'*K(:,:,k)',eye(nX)) - kron(eye(nX),G(:,:,k)*H'*K(:,:,k)')*comm(nX,nU);
                        dEdG = kron(eye(nX),(A(:,:,k)-B(:,:,k)*K(:,:,k))*H)*comm(nX,nW) + kron((A(:,:,k)-B(:,:,k)*K(:,:,k))*H,eye(nX)) + kron(eye(nX), G(:,:,k)*obj.D)*comm(nX,nW) + kron(G(:,:,k)*obj.D, eye(nX));
                        dEdK = -kron(B(:,:,k), A(:,:,k)*E(:,:,k))*comm(nU,nX) - kron(A(:,:,k)*E(:,:,k), B(:,:,k)) + kron(B(:,:,k)*K(:,:,k)*E(:,:,k), B(:,:,k)) + kron(B(:,:,k),B(:,:,k)*K(:,:,k)*E(:,:,k))*comm(nU,nX) - kron(G(:,:,k)*H',B(:,:,k)) - kron(B(:,:,k),G(:,:,k)*H')*comm(nU,nX);
                        dEdH = kron(G(:,:,k), A(:,:,k)-B(:,:,k)*K(:,:,k)) + kron(A(:,:,k)-B(:,:,k)*K(:,:,k), G(:,:,k))*comm(nX,nW);
                        dEdE = kron(A(:,:,k)-B(:,:,k)*K(:,:,k), A(:,:,k)-B(:,:,k)*K(:,:,k));
                        
                        dHdA = kron(H', eye(nX));
                        dHdB = -kron(H'*K(:,:,k)', eye(nX));
                        dHdG = kron(obj.D, eye(nX));
                        dHdK = -kron(H', B(:,:,k));
                        dHdH = kron(eye(nW), (A(:,:,k)-B(:,:,k)*K(:,:,k)));
                        
                        E(:,:,k+1) = (A(:,:,k)-B(:,:,k)*K(:,:,k))*E(:,:,k)*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + (A(:,:,k)-B(:,:,k)*K(:,:,k))*H*G(:,:,k)' + G(:,:,k)*H'*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + G(:,:,k)*obj.D*G(:,:,k)';
                        H = (A(:,:,k)-B(:,:,k)*K(:,:,k))*H + G(:,:,k)*obj.D;
                        
                        dE(:,:,k+1) = dEdE*dE(:,:,k) + dEdH*dH + dEdK*dK(:,:,k);
                        dE(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU)),k+1) = dE(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU)),k+1) + dEdA*dA(:,:,k) + dEdB*dB(:,:,k) + dEdG*dG(:,:,k);
                        
                        dH = dHdH*dH + dHdK*dK(:,:,k);
                        dH(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU))) = dH(:,(k-1)*(1+nX+nU)+(1:2*(1+nX+nU))) + dHdA*dA(:,:,k) + dHdB*dB(:,:,k) + dHdG*dG(:,:,k);
                    end
                    k = N-1;
                    
                    dEdA = kron(eye(nX), A(:,:,k)*E(:,:,k))*comm(nX,nX) + kron(A(:,:,k)*E(:,:,k), eye(nX)) - kron(B(:,:,k)*K(:,:,k)*E(:,:,k), eye(nX)) - kron(eye(nX), B(:,:,k)*K(:,:,k)*E(:,:,k))*comm(nX,nX) + kron(G(:,:,k)*H',eye(nX)) + kron(eye(nX),G(:,:,k)*H')*comm(nX,nX);
                    dEdB = -kron(eye(nX), A(:,:,k)*E(:,:,k)*K(:,:,k)')*comm(nX,nU) - kron(A(:,:,k)*E(:,:,k)*K(:,:,k)', eye(nX)) + kron(eye(nX), B(:,:,k)*K(:,:,k)*E(:,:,k)*K(:,:,k)')*comm(nX,nU) + kron(B(:,:,k)*K(:,:,k)*E(:,:,k)*K(:,:,k)', eye(nX)) - kron(G(:,:,k)*H'*K(:,:,k)',eye(nX)) - kron(eye(nX),G(:,:,k)*H'*K(:,:,k)')*comm(nX,nU);
                    dEdG = kron(eye(nX),(A(:,:,k)-B(:,:,k)*K(:,:,k))*H)*comm(nX,nW) + kron((A(:,:,k)-B(:,:,k)*K(:,:,k))*H,eye(nX)) + kron(eye(nX), G(:,:,k)*obj.D)*comm(nX,nW) + kron(G(:,:,k)*obj.D, eye(nX));
                    dEdK = -kron(B(:,:,k), A(:,:,k)*E(:,:,k))*comm(nU,nX) - kron(A(:,:,k)*E(:,:,k), B(:,:,k)) + kron(B(:,:,k)*K(:,:,k)*E(:,:,k), B(:,:,k)) + kron(B(:,:,k),B(:,:,k)*K(:,:,k)*E(:,:,k))*comm(nU,nX) - kron(G(:,:,k)*H',B(:,:,k)) - kron(B(:,:,k),G(:,:,k)*H')*comm(nU,nX);
                    dEdH = kron(G(:,:,k), A(:,:,k)-B(:,:,k)*K(:,:,k)) + kron(A(:,:,k)-B(:,:,k)*K(:,:,k), G(:,:,k))*comm(nX,nW);
                    dEdE = kron(A(:,:,k)-B(:,:,k)*K(:,:,k), A(:,:,k)-B(:,:,k)*K(:,:,k));
                    
                    E(:,:,k+1) = (A(:,:,k)-B(:,:,k)*K(:,:,k))*E(:,:,k)*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + (A(:,:,k)-B(:,:,k)*K(:,:,k))*H*G(:,:,k)' + G(:,:,k)*H'*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + G(:,:,k)*obj.D*G(:,:,k)';
                    
                    dE(:,:,k+1) = dEdE*dE(:,:,k) + dEdH*dH + dEdK*dK(:,:,k);
                    dE(:,(k-1)*(1+nX+nU)+(1:1+nX+nU),k+1) = dE(:,(k-1)*(1+nX+nU)+(1:1+nX+nU),k+1) + dEdA*dA(:,1:(1+nX+nU),k) + dEdB*dB(:,1:(1+nX+nU),k) + dEdG*dG(:,1:(1+nX+nU),k);
                    dE(:,k*(1+nX+nU)+(1:nX),k+1) = dE(:,k*(1+nX+nU)+(1:nX),k+1) + dEdA*dA(:,(1+nX+nU+1)+(1:nX),k) + dEdB*dB(:,(1+nX+nU+1)+(1:nX),k) + dEdG*dG(:,(1+nX+nU+1)+(1:nX),k);
            end
            
            obj.K_handle.data = K;
            obj.A_handle.data = A;
            obj.B_handle.data = B;
            obj.G_handle.data = G;
            obj.E_handle.data = E;
            obj.dK_handle.data = dK;
            obj.dA_handle.data = dA;
            obj.dB_handle.data = dB;
            obj.dG_handle.data = dG;
            obj.dE_handle.data = dE;
            return
        else
            K = obj.K_handle.data;
            A = obj.A_handle.data;
            B = obj.B_handle.data;
            G = obj.G_handle.data;
            E = obj.E_handle.data;
            dK = obj.dK_handle.data;
            dA = obj.dA_handle.data;
            dB = obj.dB_handle.data;
            dG = obj.dG_handle.data;
            dE = obj.dE_handle.data;
            return
        end
    end
    
    function [f,df,d2f] = robust_dynamics(obj,h,x,u,w)
      % Euler integration of continuous dynamics
      if nargout == 1
          xdot = obj.plant.dynamics_w(0,x,u,w);
          f = x + h*xdot;
      elseif nargout == 2
        [xdot,dxdot] = obj.plant.dynamics_w(0,x,u,w);
        f = x + h*xdot;
        df = [xdot ... h
          eye(obj.nX) + h*dxdot(:,1+(1:obj.nX)) ... x0
          h*dxdot(:,1+obj.nX+(1:obj.nU)) ... u
          h*dxdot(:,1+obj.nX+obj.nU+(1:obj.nW))]; % w
      else %nargout == 3
          [xdot,dxdot,d2xdot] = obj.plant.dynamics_w(0,x,u,w);
          f = x + h*xdot;
          df = [xdot ... h
            eye(obj.nX) + h*dxdot(:,1+(1:obj.nX)) ... x0
            h*dxdot(:,1+obj.nX+(1:obj.nU)) ... u
            h*dxdot(:,1+obj.nX+obj.nU+(1:obj.nW))]; % w
          d2f = h*d2xdot;
          d2f(:,1:(1+obj.nX+obj.nU+obj.nW)) = dxdot;
          d2f(:,1:(1+obj.nX+obj.nU+obj.nW):end) = dxdot;
      end
    end
    
    function utraj = reconstructInputTrajectory(obj,z)
      % zero-order holds
      t = [0; cumsum(z(obj.h_inds(1:end-1)))];
      if size(obj.u_inds,1)>0
        u = reshape(z(obj.u_inds),[],obj.N-1);
        utraj = PPTrajectory(zoh(t,u));
        utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
      else
        utraj=[];
      end
    end

    function xtraj = reconstructStateTrajectory(obj,z)
      % first-order holds
      t = [0; cumsum(z(obj.h_inds))];
      x = reshape(z(obj.x_inds),[],obj.N);
      xtraj = PPTrajectory(foh(t,x));
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
    end
    
    function [S, T] = fastsqrt(obj,A)
        %FASTSQRT computes the square root of a matrix A with Denman-Beavers iteration
        
        %S = sqrtm(A);
        
        if nnz(diag(A) > 0) ~= size(A,1)
            S = diag(sqrt(diag(A)));
            return
        end
        
        I = eye(size(A,1));
        S = A;
        T = I;
        
        T = .5*(T + inv(S));
        S = .5*(S+I);
        for k = 1:4
            Snew = .5*(S + inv(T));
            T = .5*(T + inv(S));
            S = Snew;
        end
        
    end

%     function [c, dc] = robust_state_constraint_fd(obj,y,xf,constr_fun,~,x_ind)
%         nX = obj.nX;
%         nU = obj.nU;
%         nW = obj.nW;
%         N = obj.N;
%         nXc = length(x_ind); %number of constrained components
%         nC = 2*(N-2)*nXc; %number of constraints
% 
%         delta = 1e-7;
%         
%         c = robust_state_constraint_1(obj,y,xf,constr_fun,x_ind);
%         
%         dc = zeros(nC,length(y)+length(xf));
%         dy = zeros(size(y));
%         for k = 1:length(y)
%             dy(k) = delta;
%             dc(:,k) = (robust_state_constraint_1(obj,y+dy,xf,constr_fun,x_ind) - robust_state_constraint_1(obj,y-dy,xf,constr_fun,x_ind))./(2*delta);
%             %dc(:,k) = (robust_state_constraint_1(obj,y+dy,xf,constr_fun,x_ind) - c)/delta;
%             dy(k) = 0;
%         end
%         dxf = zeros(size(xf));
%         for k = 1:length(xf)
%             dxf(k) = delta;
%             dc(:,length(y)+k) = (robust_state_constraint_1(obj,y,xf+dxf,constr_fun,x_ind) - robust_state_constraint_1(obj,y,xf-dxf,constr_fun,x_ind))./(2*delta);
%             %dc(:,length(y)+k) = (robust_state_constraint_1(obj,y,xf+dxf,constr_fun,x_ind) - c)/delta;
%             dxf(k) = 0;
%         end
%     end
%     
%     function c = robust_state_constraint_1(obj,y,xf,constr_fun,x_ind)
%         nX = obj.nX;
%         nU = obj.nU;
%         nW = obj.nW;
%         N = obj.N;
%                 
%         nCout = constr_fun.num_cnstr;
%         nXc = length(x_ind); %number of constrained components
%         nC = 2*nCout*(N-2)*nXc; %number of constraints
%         Pc = obj.Pc; %projection onto constrained subspace
%         
%         v = robust_state_constraint_v(obj,y,xf,constr_fun,x_ind);
%         
%         %Evaluate constraint function
%         xinds = kron(1+(1:N-2)*(1+nX+nU), ones(length(x_ind),1)) + kron(ones(1,N-2), x_ind(:));
%         xp = y(xinds);
%         x = kron(xp, ones(1,nXc));
%         xc = [x+v x-v];
%         
%         c = zeros(constr_fun.num_cnstr,nC);
%         for k = 1:nC
%             c(:,k) = constr_fun.eval(xc(:,k));
%         end
%         
%         c = c(:);
%     end
%
%     function c = robust_cost_1(obj,y,xf)
%         nX = obj.nX;
%         nU = obj.nU;
%         nW = obj.nW;
%         N = obj.N;
%         
%         [K,A,B,G] = lqrController(obj,y,xf);
%         
%         c = 0;
%         E = obj.E0;
%         H = zeros(nX,nW);
%         for k = 1:(N-1)
%             c = c + trace((obj.Qr + K(:,:,k)'*obj.Rr*K(:,:,k))*E);
%             
%             E = (A(:,:,k)-B(:,:,k)*K(:,:,k))*E*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + (A(:,:,k)-B(:,:,k)*K(:,:,k))*H*G(:,:,k)' + G(:,:,k)*H'*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + G(:,:,k)*obj.D*G(:,:,k)';
%             H = (A(:,:,k)-B(:,:,k)*K(:,:,k))*H + G(:,:,k)*obj.D;
%         end
%         c = c + trace(obj.Qrf*E);
%     end
%     
%     function [c, dc] = robust_cost_fd(obj,y,xf)
%         nX = obj.nX;
%         nU = obj.nU;
%         nW = obj.nW;
%         N = obj.N;
%         
%         [K,A,B,G] = lqrController(obj,y,xf);
%         
%         c = 0;
%         E = obj.E0;
%         H = zeros(nX,nW);
%         for k = 1:(N-1)
%             c = c + trace((obj.Qr + K(:,:,k)'*obj.Rr*K(:,:,k))*E);
%             
%             E = (A(:,:,k)-B(:,:,k)*K(:,:,k))*E*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + (A(:,:,k)-B(:,:,k)*K(:,:,k))*H*G(:,:,k)' + G(:,:,k)*H'*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + G(:,:,k)*obj.D*G(:,:,k)';
%             H = (A(:,:,k)-B(:,:,k)*K(:,:,k))*H + G(:,:,k)*obj.D;
%         end
%         c = c + trace(obj.Qrf*E);
%         
%         delta = 5e-7;
%         dc = zeros(1,length(y)+length(xf));
%         dy = zeros(size(y));
%         for k = 1:length(y)
%             dy(k) = delta;
%             dc(k) = (robust_cost_1(obj,y+dy,xf) - robust_cost_1(obj,y-dy,xf))/(2*delta);
%             dy(k) = 0;
%         end
%         dxf = zeros(size(xf));
%         for k = 1:length(xf)
%             dxf(k) = delta;
%             dc(length(y)+k) = (robust_cost_1(obj,y,xf+dxf) - robust_cost_1(obj,y,xf-dxf))/(2*delta);
%             dxf(k) = 0;
%         end
%     end
% 
%     function [c, dc] = robust_input_constraint_fd(obj,y,xf)
%         nX = obj.nX;
%         nU = obj.nU;
%         N = obj.N;
%         delta = 1e-5;
%         
%         c = robust_constraint_1(obj,y,xf);
%         
%         dc = zeros(2*(N-2)*nU,length(y)+length(xf));
%         dy = zeros(size(y));
%         for k = 1:length(y)
%             dy(k) = delta;
%             dc(:,k) = (robust_constraint_1(obj,y+dy,xf) - robust_constraint_1(obj,y-dy,xf))./(2*delta);
%             dy(k) = 0;
%         end
%         dxf = zeros(size(xf));
%         for k = 1:length(xf)
%             dxf(k) = delta;
%             dc(:,length(y)+k) = (robust_constraint_1(obj,y,xf+dxf) - robust_constraint_1(obj,y,xf-dxf))./(2*delta);
%             dxf(k) = 0;
%         end
%     end
%     
%     function c = robust_input_constraint_1(obj,y,xf)
%         nX = obj.nX;
%         nU = obj.nU;
%         nW = obj.nW;
%         N = obj.N;
%         
%         [K,A,B,G] = lqrController(obj,y,xf);
%         
%         v = zeros((N-2)*nU,nW);
%         E = G(:,:,1)*obj.D*G(:,:,1)';
%         H = G(:,:,1)*obj.D;
%         for k = 2:(obj.N-1)
%             U = K(:,:,k)*E*K(:,:,k)';
%             v((k-2)*(nU*nU)+(1:nU*nU)) = vec(U^(1/2));
%             
%             E = (A(:,:,k)-B(:,:,k)*K(:,:,k))*E*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + (A(:,:,k)-B(:,:,k)*K(:,:,k))*H*G(:,:,k)' + G(:,:,k)*H'*(A(:,:,k)-B(:,:,k)*K(:,:,k))' + G(:,:,k)*obj.D*G(:,:,k)';
%             H = (A(:,:,k)-B(:,:,k)*K(:,:,k))*H + G(:,:,k)*obj.D;
%         end
%         
%         u = y(1+nX+(1:N-2)'*(1+nX+nU)+kron(ones(N-2,1), (1:nU)'));
%         uc = kron(ones(nU,1), u);
%         c = [uc+v(:); uc-v(:)];
%     end
  end
end