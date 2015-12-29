classdef PseudoSpectralMethodTrajOpt <DirectTrajectoryOptimization
  % Pseudospectral (PS) method for trajectory optimization

  % uses a global polynomial to approximate the dynamics. The global polynomial is obtained from Lagrange interpolation with chosen basis polynomial from
  % the gloablly orthogonal polynomial family, e.g. Legendre and Chebyshev polynomials, and the interpolation points are the affine transformations of 
  % Gauss quadratures 

  % this class offers two specifications of the PS method family
  % to use Legendre polynomials at Lengendre-Gauss-Lobatto quadratures (LGL), set option.ps_method=PseudoSpectralMethodTrajOpt.LGL  % see Elnagar et al., 'The Pseudospectral Legendre Method for Discretizing Optimal Control Problems', 1995, for the underlying math formulation for LGL method,
  
  % to use Chebyshev polynomials at Chebyshev-Gauss-Lobatto quadratures (CGL), set option.ps_method=PseudoSpectralMethodTrajOpt.CGL
  % and see Fahroo et al., 'Direct Trajectory Optimization by a Chebyshev Pseudospectral Method', 2002, for the math formulation for CGL method
  
  % see runPSM in /drake/examples/glider for an example call of this method

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
      if options.ps_method~=PseudoSpectralMethodTrajOpt.LGL&options.ps_method~=PseudoSpectralMethodTrajOpt.CGL
        error('Drake:PseudoSpectralMethodTrajOpt:InvalidArgument','Unknown Pseudospectral Method');
      end
      if ~isfield(options,'time_option')| options.time_option ~= 3
        % force time_option to skip option 1 and 2, so to not add time constraint from the superclass
        % time constraint will be added later in this class as it is unique to PS method
        options.time_option = 3;
      end

      obj = obj@DirectTrajectoryOptimization(plant,N,durations,options);

      % add time constraint
      % all timesteps are affine transformations from Gauss quadratures
      % only used for PS, so superclass is not overwriteen
      initial_h=diff(obj.tau);
      ratio=initial_h/initial_h(1);
      A_time = ones(1,N-1); % so the sum of all time steps will be bounded between durations bounds
      S_time=[ratio,zeros(N-1,N-2)]-eye(N-1);  % so the time steps will keep the inter-point ratio as the initial one obtained from Gauss quadatures
      scale_constaint = LinearConstraint([durations(1);-inf; zeros(N-2,1)],[durations(2);inf;zeros(N-2,1)], [A_time;S_time]);
      obj = obj.addConstraint(scale_constaint,obj.h_inds);  
    end

    function obj = addDynamicConstraints(obj)
      N = obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();

      % set up psudospectral method parameters
       switch obj.options.ps_method
        case PseudoSpectralMethodTrajOpt.LGL 
          obj.tau=LGL_nodes(obj);
          obj.D=LGL_Dmatrix(obj.tau);
          obj.w=LGL_weights(obj.tau);
        case PseudoSpectralMethodTrajOpt.CGL 
          obj.tau=CGL_nodes(obj);
          obj.D=CGL_Dmatrix(obj.tau);
          obj.w=CGL_weights(obj.tau);
        otherwise
          error('Drake:PseudoSpectralMethodTrajOpt:InvalidArgument','Unknown Pseudospectral Method');
      end

      % collocation at each and every knot point requires the states and input values at all knot points. Hence n_vars has this N*(nx+nU) part
      % it also requires one of the time steps to keep the ratio correct, here arbitrarily chooses h1, so total n_vars=N*(nx+nU)+1      n_vars = 1+(N)*(nX+nU);
      n_vars = 1+(N)*(nX+nU);
      cnstr = FunctionHandleConstraint(zeros(N*(nX),1),zeros(N*(nX),1),n_vars,@obj.constraint_fun);
      cnstr = setName(cnstr,'PSMcollocation');

      % set up sparse structure
      % the constraint gradient matrix always have this general structure
      gradient_structure=[ones(N*nX,1),kron(ones(N,N),ones(nX,nX)),kron(ones(N,N),ones(nX,nU))];
      % find the spasity indices
      [sparse_Row,sparse_Col] = find(gradient_structure~=0); 
      cnstr = setSparseStructure(cnstr,sparse_Row,sparse_Col);

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
      % @param h1, the first time step, used for keeping time-derivative from dynamics at the right scale
      % @param x, state, at the current knot of consideration
      % @param u, input, at the current knot of consideration
      % @param varargin, a cell of system dynamics and their derivatives at all knot points, i.e., size(varagin)=obj.N
      
      % initilize parameters
      N=obj.N;
      nX = obj.plant.getNumStates();
      nU = obj.plant.getNumInputs();
      D=obj.D;
      initial_h1=obj.tau(2)-obj.tau(1);
      scale=h1/(initial_h1);% comes from the time domain affine transformation
      x=reshape(x,nX,N);

      % the collocation
      % subscript p indicates the parameters are from the psm method,f indicates they're from the dynamics
      % all dynamics data
      dynamics=cell2mat(varargin);
      % raw data of xdot (f) obtained at all knot points
      xdot_all_knots=vertcat(dynamics.xdot);
      % xdot from the dynamics, with scaling
      xdot_f=xdot_all_knots*scale;
      % xdot from PS approximation
      xdot_p=sum((kron(D,ones(nX,1))).*(repmat(x,N,1)),2);

      % the gradients
      % dxdot/dh1, from dynamics, in essense comes from the scaling. Not to be confused with dxdot/dt,
      % which describes the time derivative
      dxdotdh1_f=xdot_all_knots/initial_h1;
      % raw data of dxdot, which contains [dxdot/dt, dxdot/dx, dxdot/du]. For time-invariant systems,
      % the dxdot/dt part is always 0, and thus omitted
      dxdot_all_knots=vertcat(dynamics.dxdot);
      % raw data of dxdot/dx
      dxdotdx_all_knots=dxdot_all_knots(:,2:1+nX);
      % processing of the raw data, essentially transformed the raw data into a block diagonal matrix
      dxdotdx_f=(repmat(dxdotdx_all_knots,1,N)).*kron(eye(N),ones(nX,nX))*scale; 
      % raw data of dxdot/du
      dxdotdu_all_knots=dxdot_all_knots(:,2+nX:end);
      % similar processing procedure
      dxdotdu_f=(repmat(dxdotdu_all_knots,1,N)).*kron(eye(N),ones(nX,nU))*scale;
      % d(xdot_p)/dx
      dxdotdx_p=kron(D,speye(nX)); 

      % difference between the dynamic and the psm fitting curve
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
      % the weight vector
      w=obj.w;

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
      % Interpolate all knot points to reconstruct a trajectory using
      % Lagrange interpolation
      nU = obj.plant.getNumInputs();
      u = reshape(z(obj.u_inds),[],obj.N);
      % it is tempting to interpolate at the physical time domain nodes as it is the most convenient, but to minimize the effect of Runge phenomenon, 
      % do the Lagrange interpolation at tau (Gauss quadratures). The shifting will be taken care of in LagrangeInterpolation method.
      % note that, while this remedy does minimize the effect, it unfortunately doesn't gurantee the Runge phenomenon won't happen. Polyfit call will give warning
      % when Runge phenomenon is unavoidable.
      % a degenerated PPTrajectory, in the sense that there's only one break
      utraj=PPTrajectory(LagrangeInterpolation(obj.tau,u,nU));
      % scale from [0,2] to the physical time domain
      utraj = utraj.scaleTime(sum(z(obj.h_inds))/2);
      utraj = utraj.setOutputFrame(obj.plant.getInputFrame);
    end
    
    function xtraj = reconstructStateTrajectory(obj,z)
      % Interpolate all knot points to reconstruct a trajectory using
      % Lagrange interpolation
      nX = obj.plant.getNumStates();
      x = reshape(z(obj.x_inds),[],obj.N);
      % similar reason as in reconstructInputTrajectory() for the choice of interpolating times
      xtraj=PPTrajectory(LagrangeInterpolation(obj.tau,x,nX));
      % scale from [0,2] to te physical time domain
      xtraj = xtraj.scaleTime(sum(z(obj.h_inds))/2);
      xtraj = xtraj.setOutputFrame(obj.plant.getStateFrame);
    end
  end
  
  methods (Access = protected)
  
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