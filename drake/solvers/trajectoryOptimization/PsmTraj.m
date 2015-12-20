classdef PsmTraj < NonlinearProgram
  % Pseudospectral method
  % Knot pointes are determined by N-th order orthogonal polynomials, e.g. Legendre polynomials, Chebyshev polynomials
  % Then at each knot points, use Lagrange polynomials to approximate state values. 
  % f(x(k),u(k)) and f(x(k+1),u(k+1)) are evaluated at the knot points to match the derivative of the Lagrange poly
   %%options,'ps_method' detemines if to use Legendre (option.ps_method=1) or to use Chebyshev (option.ps_method=2)

  properties (SetAccess = protected)
    N       % number of timesteps
    options % options, yup
    plant   % the plant
    h_inds  % (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
    x_inds  % n x N indices for state
    u_inds  % m x N indices for input
    tau
    D
    w
    dynamic_constraints = {};
    constraints = {};
  end
  
  methods
    function obj = PsmTraj(plant,N,durations,options)
      if isscalar(durations), durations=[durations,durations]; end
      if nargin < 4
        options = struct();
      end
      if ~isfield(options,'ps_method')
        options.ps_method = 1;
      end
      if ~plant.isTI
        error('Drake:DirectTrajectoryOptimization:UnsupportedPlant','Only time-invariant plants are currently supported');
      end

      obj = obj@NonlinearProgram(0);
      obj.options = options;
      obj.plant = plant;
      obj = obj.setupVariables(N);
      

      initial_h=diff(obj.tau);%N-1 row,
      ratio=zeros(N-1,1);
      for i=1:N-1;
        ratio(i)= initial_h(i)/initial_h(1);
      end
      A_time = ones(1,N-1); % so the sum of all time steps will be bounded between durations bounds
      S_time=[ratio,zeros(N-1,N-2)]-eye(N-1); %so the time steps will keep the initial ratio from Gauss quadatures
      scale_constaint = LinearConstraint([durations(1);-inf; zeros(N-2,1)],[durations(2);inf;zeros(N-2,1)], [A_time;S_time]);
      obj = obj.addConstraint(scale_constaint,obj.h_inds);

      obj = obj.addConstraint(BoundingBoxConstraint(zeros(N-1,1),inf(N-1,1)),obj.h_inds);
      % create constraints for dynamics and add them
      obj = obj.addDynamicConstraints();
      % add control inputs as bounding box constraints
      if any(~isinf(plant.umin)) || any(~isinf(plant.umax))
        control_limit = BoundingBoxConstraint(repmat(plant.umin,N,1),repmat(plant.umax,N,1));
        obj = obj.addConstraint(control_limit,obj.u_inds(:));
      end
    end


    function obj = addDynamicConstraints(obj)
      N = obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();

      %only need the state and input value at each knot points
      n_vars = (N)*(nX+nU)+1;
      cnstr = FunctionHandleConstraint(zeros(N*(nX),1),zeros(N*(nX),1),n_vars,@obj.constraint_fun);
      cnstr = setName(cnstr,'PSMcollocation');
      % shared_data_index = obj.getNumSharedDataFunctions;
      for i=1:obj.N,
        obj = obj.addSharedDataFunction(@obj.dynamics_data,{obj.x_inds(:,i);obj.u_inds(:,i)});      
      end 

      all_x_inds=reshape(obj.x_inds(:,:),N*nX,1);
      all_u_inds=reshape(obj.u_inds(:,:),N*nU,1);
      obj = obj.addConstraint(cnstr,{obj.h_inds(1);all_x_inds;all_u_inds},[1:N]);
    end


    function [g,dg] = constraint_fun(obj,h1,x,u,varargin)
    %%necessary parameters
      N=obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      options=obj.options;
      D=obj.D;
      initial_h1=obj.tau(2)-obj.tau(1);
      scale=h1/initial_h1;%comes from the time domain affine transformation

      %%subscript p is for parameters from the psm method,f is for that from dynamics
    %%from dynamics:
      xdot_f=sparse(nX*N,1);
      dxdh_f=sparse(nX*N,1);
      dxdot_f=sparse(nX*N,nX*N);
      udot_f=sparse(nX*N,N*nU);
      dxdudot_f=sparse(N*nU,N*nX);
      dxandudot_f=sparse(N*(nX+nU),N*(nX+nU)+1);
    % from psm
      xdot_p=sparse(nX*N,1);
      u_p=sparse(nU*N,1);
      dD_p=sparse(N*(nX+nU),N*(nX+nU));
      dDxx_p=sparse(N*nX,N*nX);
      dDxu_p=sparse(N*nX,N*nU);

    %%f and df (the dynamics) evaluated at the knot points  
      for k=1:N, %row indices
        xdot_f((k-1)*nX+1:k*nX,:)= varargin{k}.xdot*scale; %%f(x,u), with scaling
        dxdh_f((k-1)*nX+1:k*nX,:)= varargin{k}.xdot/initial_h1; %%df/dh 
        dxdot_f((k-1)*nX+1:k*nX,(k-1)*nX+1:k*nX) = varargin{k}.dxdot(:,2:1+nX)*scale;%%df/dx, with scaling
        udot_f((k-1)*nX+1:k*nX,(k-1)*nU+1:k*nU) = varargin{k}.dxdot(:,2+nX:end)*scale; %%df/du 
        for j=1:N, %column indices
          xdot_p((k-1)*(nX)+1:k*nX,:)=xdot_p((k-1)*(nX)+1:k*nX,:)+D(k,j)*x((j-1)*nX+1:j*nX);
          dDxx_p((k-1)*(nX)+1:k*nX,(j-1)*(nX)+1:j*nX) = D(k,j)*eye(nX);
          dDuu_p((k-1)*(nU)+1:k*nU,(j-1)*(nU)+1:j*nU) = D(k,j)*eye(nU);
        end       
      end 
      
    %%difference between the dynamic and collocation
      g=[xdot_p]-[xdot_f];
    %% gradient of such difference
      dxandudot_f=[dxdh_f,dxdot_f,udot_f];
      dD_p=[zeros(N*nX,1),dDxx_p,dDxu_p];
      dg=sparse(dD_p)-sparse(dxandudot_f);      
    end


    function obj = addInputConstraint(obj,constraint,time_index)
      % Add constraint (or composite constraint) that is a function of the
      % input at the specified time or times.
      % @param constraint  a Constraint or CompositeConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3. For convenience, [1,2,3] is also
      %   interpreted as {1,2,3}.
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      if ~iscell(time_index)
        time_index = num2cell(reshape(time_index,1,[]));
      end
      for j=1:length(time_index),
        cstr_inds = mat2cell(obj.u_inds(:,time_index{j}),size(obj.u_inds,1),ones(1,length(time_index{j})));

        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cstr_inds;
        obj.constraints{end}.time_index = time_index;

        obj = obj.addConstraint(constraint,cstr_inds);
      end
    end

    function obj = addStateConstraint(obj,constraint,time_index,x_indices)
      % Add constraint (or composite constraint) that is a function of the
      % state at the specified time or times.
      % @param constraint  a CompositeConstraint
      % @param time_index   a cell array of time indices
      %   ex1., time_index = {1, 2, 3} means the constraint is applied
      %   individually to knot points 1, 2, and 3. For convenience, [1,2,3] is also
      %   interpreted as {1,2,3}.
      %   ex2,. time_index = {[1 2], [3 4]} means the constraint is applied to knot
      %   points 1 and 2 together (taking the combined state as an argument)
      %   and 3 and 4 together.
      % @param x_index optional subset of the state vector x which this
      % constraint depends upon @default 1:num_x
      if ~iscell(time_index)
        % then use { time_index(1), time_index(2), ... } ,
        % aka independent constraints for each time
        time_index = num2cell(reshape(time_index,1,[]));
      end
      if nargin<4, x_indices = 1:size(obj.x_inds,1); end

      for j=1:length(time_index),
        cstr_inds = mat2cell(obj.x_inds(x_indices,time_index{j}),numel(x_indices),ones(1,length(time_index{j})));

        % record constraint for posterity
        obj.constraints{end+1}.constraint = constraint;
        obj.constraints{end}.var_inds = cstr_inds;
        obj.constraints{end}.time_index = time_index;

        obj = obj.addConstraint(constraint,cstr_inds);
      end
    end

    function obj = addTrajectoryDisplayFunction(obj,display_fun)
      % add a dispay function that gets called on every iteration of the
      % algorithm
      % @param display_fun a function handle of the form displayFun(t,x,u)
      %       where t is a 1-by-N, x is n-by-N and u is m-by-N
      obj = addDisplayFunction(obj,@(z) display_fun(z(obj.h_inds),z(obj.x_inds),z(obj.u_inds)));
    end
    
    function [xtraj,utraj,z,F,info,infeasible_constraint_name] = solveTraj(obj,t_init,traj_init)
      % Solve the nonlinear program and return resulting trajectory
      % @param t_init initial timespan for solution.  can be a vector of
      % length obj.N specifying the times of each segment, or a scalar
      % indicating the final time.
      % @param traj_init (optional) a structure containing Trajectory
      % objects specifying the initial guess for the system inputs
      %    traj_init.u , traj_init.x, ...
      % @default small random numbers

      if nargin<3, traj_init = struct(); end

      z0 = obj.getInitialVars(t_init,traj_init);
      [z,F,info,infeasible_constraint_name] = obj.solve(z0);
      xtraj = reconstructStateTrajectory(obj,z);
      if nargout>1, utraj = reconstructInputTrajectory(obj,z); end
    end

   

    function obj = setupVariables(obj, N)
      % Default implementation,
      % Assumes, if time is not fixed, that there are N-1 time steps
      % N corresponding state variables
      % and N-1 corresponding input variables
      %
      % Generates num_vars total number of decision variables
      %   h_inds (N-1) x 1 indices for timesteps h so that z(h_inds(i)) = h(i)
      %   x_inds N x n indices for state
      %   u_inds N x m indices for time
      %
      % @param N number of knot points
      nH = N-1;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();

      num_vars = nH + N*(nX+nU);
      obj.h_inds = (1:nH)';
      obj.x_inds = reshape(nH + (1:nX*N),nX,N);
      obj.u_inds = reshape(nH + nX*N + (1:nU*N),nU,N);


      obj.N = N;
      x_names = cell(num_vars,1);
      for i = 1:N
        if(i<N)
          x_names{i} = sprintf('h[%d]',i);
        end
        for j = 1:nX
          x_names{nH+(i-1)*nX+j}=sprintf('x%d[%d]',j,i);
        end
        for j = 1:nU
          x_names{nH+nX*N+(i-1)*nU+j} = sprintf('u%d[%d]',j,i);
        end
      end

      obj = obj.addDecisionVariable(num_vars,x_names);
      options=obj.options;

      %%set up psudospectral method parameters
       switch options.ps_method
        case 1 % Legendre polynomial
          obj.tau=LGL_nodes(obj);
          obj.D=LGL_Dmatrix(obj.tau);
          obj.w=LGL_weights(obj.tau);
        case 2 % chebyshev polynomial
          obj.tau=CGL_nodes(obj);
          obj.D=CGL_Dmatrix(obj.tau);
          obj.w=CGL_weights(obj.tau);
      end
      % toc
    end

    function z0 = getInitialVars(obj,t_init,traj_init)
    % evaluates the initial trajectories at the sampled times and
    % constructs the nominal z0. Overwrite to implement in a different
    % manner
      if isscalar(t_init)
        t_init = .5*t_init*(ones(1,obj.N)+obj.tau');
      elseif length(t_init) ~= obj.N
        error('The initial sample times must have the same length as property N')
      end
      z0 = zeros(obj.num_vars,1);
      z0(obj.h_inds) = diff(obj.tau);
      if nargin<3, traj_init = struct(); end

      nU = getNumInputs(obj.plant);
      if isfield(traj_init,'u')
        z0(obj.u_inds) = traj_init.u.eval(t_init);
      else
        z0(obj.u_inds) = zeros(nU,obj.N);
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
    function obj = addInitialCost(obj,initial_cost)
      % Adds a cost to the initial state f(x0)
      obj = obj.addCost(initial_cost,obj.x_inds(:,1));
    end

    function obj = addFinalCost(obj,final_cost_function)
      % adds a cost to the final state and total time
      % @param final_cost_function a function handle f(T,xf)
      nX = obj.plant.getNumStates();
      nH = obj.N-1;
      cost = FunctionHandleObjective(nH+nX,@(h,x) obj.final_cost(final_cost_function,h,x));
      obj = obj.addCost(cost,{obj.h_inds;obj.x_inds(:,end)});
    end

  function [utraj,xtraj] = reconstructInputTrajectory(obj,z)
      %%todo
      %%fix the foh with global Lagrange interpolation
      t = [0; cumsum(z(obj.h_inds))];
      u = reshape(z(obj.u_inds),[],obj.N);
      utraj = PPTrajectory(foh(t,u));
      utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
    end
    
    function xtraj = reconstructStateTrajectory(obj,z)
      %%todo
      %%fix the foh with global Lagrange interpolation
      t = [0; cumsum(z(obj.h_inds))];
      u = reshape(z(obj.u_inds),[],obj.N);

      x = reshape(z(obj.x_inds),[],obj.N);
      xdot = zeros(size(x,1),obj.N);
      for i=1:obj.N,
        xdot(:,i) = obj.plant.dynamics(t(i),x(:,i),u(:,i));
      end
      xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);



    end

    function u0 = extractFirstInput(obj,z)
      % When using trajectory optimization a part of a model-predictive
      % control system, we often only need to extract u(0).  This method
      % does exactly that.
      u0 = z(obj.u_inds(:,1));
    end

    
    function data = dynamics_data(obj,x,u)
        [data.xdot,data.dxdot] = obj.plant.dynamics(0,x,u);
    end

    function tau = gettau(obj)
      tau = obj.tau;
    end

    function D = getD(obj)
      D = obj.D;
    end

    function w = getw(obj)
      w = obj.w;
    end

    function N = getN(obj)
      N = obj.N;
    end

    function x_inds = getXinds(obj)
      x_inds = obj.x_inds;
    end

    function h_inds = getHinds(obj)
      h_inds = obj.h_inds;
    end


    function obj = addRunningCost(obj,running_cost_function)
      % Adds an integrated cost to all time steps, which is
      % numerical implementation specific (thus abstract)
      % this cost is assumed to be time-invariant
      % @param running_cost_function a function handle
      %  of the form running_cost_function(dt,x,u)

      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      w=obj.w; %%wight vector

      %%add running cost at the first node
      running_cost_end = FunctionHandleObjective(nX+nU,@(x,u) obj.running_fun_end(running_cost_function,x,u));
      obj = obj.addCost(running_cost_end,{obj.x_inds(:,1);obj.u_inds(:,1)});   
      %%add running cost at the nodes in between
      for i=2:obj.N,
        running_cost_mid = FunctionHandleObjective(1+nX+nU,@(h,x,u) obj.running_fun_mid(running_cost_function,h,x,u,w(i)));
        obj = obj.addCost(running_cost_mid,{obj.h_inds(i-1);obj.x_inds(:,i);obj.u_inds(:,i)});
      end
    end
  end

  methods(Access=protected)
    % Adds an integrated cost to all time steps, which is
    % numerical implementation specific (thus abstract)
    % this cost is assumed to be time-invariant
    % @param running_cost_function a function handle
    %%draft psm-specific running cost handler.
    function [f,df] = running_fun_mid(obj,running_handle,h,x,u,weight)
      %%in case the running cost involves time, force the time steps to be weighted down in the discrete evalaution, and scale back up in the cost.
      %% the end result is that df in terms of time won't have any scaling factore, whereas state and input cost will be weighted according to the sequence of 
      %% the node number to get weighted accordningly.
      [g,dg] = running_handle(h/weight,x,u);
      f=g*weight;
      df = [dg(:,1), weight*dg(:,2:end)];
    end

    function [f,df] = running_fun_end(obj,running_handle,x,u)
      %%involing only first node, the subtlety here is that h(0) is always 0, i.e., the cost is indepedent of the starting time. So, feed the running_handle with 0 as the constant time-step. 
      [f,dg] = running_handle(0,x,u);
      df = dg(:,2:end);
    end

    function [f,df] = final_cost(obj,final_cost_function,h,x)
      T = sum(h);
      [f,dg] = final_cost_function(T,x);
      df = [repmat(dg(:,1),1,length(h)) dg(:,2:end)];
    end


  end
end
    

