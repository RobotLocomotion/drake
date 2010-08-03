classdef Dynamics
% Dynamics class
%   Encapsulates a dynamics function, xdot = f(t,x,u) and all of the
%   supporting methods including initial conditions and ODE integration.  
  
  methods 
    function obj = Dynamics(num_states,num_inputs)
      % Dynamics class constructor
      %   sets the dimension of the state and input vectors
      obj.m_num_states = num_states;
      obj.m_num_inputs = num_inputs;
    end
  end
  
  % methods that MUST be implemented
  methods (Abstract=true)
    xdot = dynamics(obj,t,x,u);  % implements xdot = f(t,x,u)
  end
  
  % methods that CAN be implemented/overridden
  methods

    function df = dynamicsGradients(obj,t,x,u,order)
      % dynamicsGradients
      %   computes the gradients of the dynamics around a point, to a
      %   specific order.

      % todo: do symbolic gradients here?
      error('not implemented yet');
    end
    
    function obj = setNumStates(num_states)
      % setNumStates
      %   a simple guard since inconsistencies can arise if the number of
      %   states is changed after construction
      if (num_states<1) error('Dynamics objects must have at least one state'); end
      obj.m_num_states = num_states;
    end
    
    function x = getInitialState(obj)
      % getInitialState
      %   returns the default initial conditions

      x = zeros(obj.m_num_states,1);
    end
    
    function obj = setNumInputs(num_inputs)
      % setNumInputs
      %   a simple guard since inconsistencies can arise if the number of
      %   inputs is changed after construction.
      if (num_inputs<0) error('num_inputs must be >=0'); end
      obj.m_num_inputs = num_inputs;
    end
    
    function u = getDefaultInput(obj)
      % getDefaultInput
      %   define the default initial conditions so that behavior is
      %   well-defined even if no controller is involved at startup.
      u = zeros(obj.m_num_inputs,1);
    end
    
    function [v dvdy] = stateVectorDiff(obj,X,y)
      % stateVectorDiff
      %   Many algorithms need to take a vector between two points in state
      %   space.  This can be complicated by things like wrapping coordinate
      %   systems.  All of those details for a given system should be
      %   encapsulated in the stateVectorDiff method.  The default behavior
      %   implemented here simply subtracts the two vectors.
      %
      %   usage:  v = stateVectorDiff(obj,X,y) computes v(:,i) = X(:,i)-y
      %     optional second output dvdy is the gradients of this vector.
      %     Note: to return dvdy, X must be a column vector.
      %

      if (size(X,1)~=obj.m_num_states) error('X should have num_states rows'); end
      if (size(y,1)~=obj.m_num_states) error('y should have num_states rows'); end
      if (size(y,2)~=1) error('y can only have a single column'); end
        
      v = X-repmat(y,1,size(X,2));
      if (nargout>1)
        if (size(X,2)>1) error('gradients only for column vector inputs'); end
        dvdy = -eye(length(y));
      end
    end
    
    function d = stateVectorNorm(obj,X,y)
      % stateVectorNorm
      %   Computes the distance between points in state space.  The default
      %   behavior implemented here is the Euclidean distance on the
      %   stateVectorDiff.

      % note: input error checking with happen immediately in stateVectorDiff
      v = stateVectorDiff(obj,X,y);
      d = sqrt(sum(v.^2,1));
    end
  end

  properties (SetAccess=private, GetAccess = public)
    m_num_states
    m_num_inputs
  end
  
  properties
    m_umin=-inf
    m_umax=inf
    m_ode_solver=@ode45;
    m_ode_options=odeset;
  end
  
end