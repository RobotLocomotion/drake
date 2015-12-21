classdef PseudoSpectralMethodTrajOpt <DirectTrajectoryOptimization
  % Pseudospectral (PS) method for trajectory optimization

  % uses a global polynomial to approximate the dynamics. The global polynomial is obtained from Lagrange interpolation with chosen basis polynomial from
  % the gloablly orthogonal polynomial family, e.g. Legendre and Chebyshev polynomials, and the interpolation points are the affine transformations of 
  % Gauss quadratures 

  % this class offers two specifications of the PS method family
  % to sue Legendre polynomials at Lengendre-Gauss-Lobatto quadratures (LGL), set option.ps_method=1
  % see Elnagar et al., 'The Pseudospectral Legendre Method for Discretizing Optimal Control Problems', 1995, for the underlying math formulation for LGL method,

  % to use Chebyshev polynomials at Chebyshev-Gauss-Lobatto quadratures (CGL), set option.ps_method=2.
  % and see Fahroo et al., 'Direct Trajectory Optimization by a Chebyshev Pseudospectral Method', 2002, for the math formulation for CGL method
  properties(SetAccess = protected)
    tau % Gauss node points, distributed between [-1,1] (linked with h(i) by affine transformation)
    D % differentiation matrix of the chosen Lagrange interpolation methods
    w % weight vector of the chosen Lagrange interpolation methods
  end
  % CGL and LGL tau, D, and w used in this method are solved for using methods obtained from third party, and they are placed in /drake/thirdParty/psm. 
  % third party functions have been tested against wolframalpha for their accuracy
  properties (Constant)
    LGL = 1;% Legendre polynomials on LGL nodes % DEFAULT
    CGL = 2;% chebyshev polynomial on CGL nodes
  end
  
  methods
    function obj = PseudoSpectralMethodTrajOpt(plant,N,durations,options)

      if nargin < 4
        options = struct();
      end
      if ~isfield(options,'ps_method')
        options.ps_method = PseudoSpectralMethodTrajOpt.LGL;
      end
      if ~isfield(options,'time_option')| options.time_option ~= 3
         % force time_option to skip option 1 and 2, to not add no time constraint from superclass
         % time constraint will be added in this class.
        options.time_option = 3;
      end

      obj = obj@DirectTrajectoryOptimization(plant,N,durations,options);

      % add time constraint
      % all timesteps are affine transformations from Gauss quadratures
      % only used for PS, so the superclass is not overwriteen
      initial_h=diff(obj.tau);
      ratio=initial_h/initial_h(1);
      A_time = ones(1,N-1); % so the sum of all time steps will be bounded between durations bounds
      S_time=[ratio,zeros(N-1,N-2)]-eye(N-1); % so the time steps will keep the inter-point ratio as the initial one obtained from Gauss quadatures
      scale_constaint = LinearConstraint([durations(1);-inf; zeros(N-2,1)],[durations(2);inf;zeros(N-2,1)], [A_time;S_time]);
      obj = obj.addConstraint(scale_constaint,obj.h_inds);  
    end

    function obj = addDynamicConstraints(obj)
      N = obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      options=obj.options;
      %%set up psudospectral method parameters
       switch options.ps_method
        case PseudoSpectralMethodTrajOpt.LGL 
          obj.tau=LGL_nodes(obj);
          obj.D=LGL_Dmatrix(obj.tau);
          obj.w=LGL_weights(obj.tau);
        case PseudoSpectralMethodTrajOpt.CGL 
          obj.tau=CGL_nodes(obj);
          obj.D=CGL_Dmatrix(obj.tau);
          obj.w=CGL_weights(obj.tau);
      end

      % collocation at each and every knot point requires the states and input values at all knot points. Hnce n_vars has this N*(nx+nU) part
      % also requires one of the time steps to keep the ratio correct, arbitrarily chooses h1, so total n_vars=N*(nx+nU)+1
      n_vars = 1+(N)*(nX+nU);
      cnstr = FunctionHandleConstraint(zeros(N*(nX),1),zeros(N*(nX),1),n_vars,@obj.constraint_fun);
      % sparse_Row=[1:N,]; 
      % sparse_Col=[ones(N),];
      % cnstr = setSparseStructure(cnstr,sparse_Row,sparse_Col)
      cnstr = setName(cnstr,'PSMcollocation');
      % system dynamics passed to collocation as sharedDataFunction
      for i=1:obj.N,
        obj = obj.addSharedDataFunction(@obj.dynamics_data,{obj.x_inds(:,i);obj.u_inds(:,i)});      
      end 
      all_x_inds=reshape(obj.x_inds(:,:),N*nX,1);
      all_u_inds=reshape(obj.u_inds(:,:),N*nU,1);
      obj = obj.addConstraint(cnstr,{obj.h_inds(1);all_x_inds;all_u_inds},[1:N]);
    end
    
    function [g,dg] = constraint_fun(obj,h1,x,u,varargin)
      % dynamic constraint
      % @param h1, the first time step, used for keeping time-derivative of PSM approximation at the right scale
      % @param x, state, at knot number i, where i is wraped in from line 79
      % @param u, input 
      % @param varargin, a cell of system dynamics at all knot points, i.e., size(varagin)=obj.N
      %initilize parameters
      N=obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      D=obj.D;
      initial_h1=obj.tau(2)-obj.tau(1);
      scale=h1/(initial_h1);%comes from the time domain affine transformation
      x=reshape(x,nX,N);


      %subscript p indicates the parameters are from the psm method,f indicates they're from the dynamics
      dynamics=cell2mat(varargin);% all dynamics data
      xdot_all_knots=vertcat(dynamics.xdot);
      xdot_f=xdot_all_knots*scale;% xdot from the dynamics
      dxdotdh1_f=xdot_all_knots/initial_h1;% dxdot/dh1, also from dynamics

      dxdot_all_knots=vertcat(dynamics.dxdot);
      dxdotdx_all_knots=dxdot_all_knots(:,2:1+nX);
      dxdotdu_all_knots=dxdot_all_knots(:,2+nX:end);
      dxdotdx_f=(repmat(dxdotdx_all_knots,1,N)).*kron(eye(N),ones(nX,nX))*scale;
      dxdotdu_f=(repmat(dxdotdu_all_knots,1,N)).*kron(eye(N),ones(nX,nU))*scale;

      xdot_p=sum((kron(D,ones(nX,1))).*(repmat(x,N,1)),2);% xdot from PS approximation
      dxdotdx_p=kron(D,eye(nX));% d(xdot_p)/dx 

      % difference between the dynamic and time-derivative of the psm fitting curve
      g=[xdot_p]-[xdot_f];
      % gradient of such difference
      d_p=[sparse(N*nX,1),dxdotdx_p,sparse(nX*N,nU*N)]; % [dxdot_p/dh1, dxdot_p/dx,dxdot_p/du]
      d_f=[dxdotdh1_f,dxdotdx_f,dxdotdu_f]; % dynamics gradient 
      dg=d_p-d_f;      
    end
    
    
    function data = dynamics_data(obj,x,u)
      [data.xdot,data.dxdot] = obj.plant.dynamics(0,x,u);
    end
    
    function obj = addRunningCost(obj,running_cost_function)
      % Adds an integrated cost to all time steps, which is
      % numerical implementation specific 
      % this cost is assumed to be time-invariant
      % @param running_cost_function a function handle
      %  of the form running_cost_function(dt,x,u)

      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      w=obj.w; %wight vector

      % add running cost at the first node
      running_cost_end = FunctionHandleObjective(nX+nU,@(x,u) obj.running_fun_end(running_cost_function,x,u));
      obj = obj.addCost(running_cost_end,{obj.x_inds(:,1);obj.u_inds(:,1)});   
      % add running cost at nodes 2 to N
      for i=2:obj.N,
        running_cost_mid = FunctionHandleObjective(1+nX+nU,@(h,x,u) obj.running_fun_mid(running_cost_function,h,x,u,w(i)));
        obj = obj.addCost(running_cost_mid,{obj.h_inds(i-1);obj.x_inds(:,i);obj.u_inds(:,i)});
      end
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
      %%fix the pchipDeriv with global Lagrange interpolation
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
  end
  
  methods (Access = protected)
    % psm-specific running cost handler.

    function [f,df] = running_fun_end(obj,running_handle,x,u)
    % running cost at the first node, which has the subtlety that is this knot point is implicity assosiated with the time step h=0
    % so feed the running_handle with a constant 0 as the time step instead of passing a variable
      [f,dg] = running_handle(0,x,u);
      df = dg(:,2:end);
    end

    function [f,df] = running_fun_mid(obj,running_handle,h,x,u,weight)
    % running cost at nodes 2 to N.
    % in case the running cost involves time, force the time steps to be weighted down in the discrete evalaution, and scale back up in the cost.
    % the end result is that the cost in term of time and df/dt won't be weighted, but the cost involving states and inputs will.
    % this trick won't work if the cost is time-variant, i.e., if the cost has terms like u*dt or x*dt
      [g,dg] = running_handle(h/weight,x,u);
      f=g*weight;
      df = [dg(:,1), weight*dg(:,2:end)];
    end
    
  end
end