classdef Dynamics
% An abstract class which wraps a dynamics function xdot=f(t,x,u).
%   Provides a common interface and many supporting methods to work with 
%   the dynamics. 
%


  methods 
    function obj = Dynamics(num_states,num_inputs)
      % Dynamics class constructor
      %   sets the dimension of the state and input vectors
      if (nargin>0)
        obj = setNumStates(obj,num_states);
        obj = setNumInputs(obj,num_inputs);
      end
    end
  end
  
  % methods that MUST be implemented
  methods (Abstract=true)
    xdot = dynamics(obj,t,x,u);  % implements xdot = f(t,x,u)
  end
  
  % methods that CAN be implemented/overridden
  methods

    function df = dynamicsGradients(obj,t,x,u,order)
      % Computes the Taylor-expansion of the dynamics around a point.
 
      % todo: implement symbolic gradients here?
      error('not implemented yet');
    end
    
    function obj = setNumStates(obj,num_states)
      % Guards the num_states variable
      if (num_states<1) error('Dynamics objects must have at least one state'); end
      obj.num_states = num_states;
    end
    
    function x = getInitialState(obj)
      % Returns the default initial conditions

      x = zeros(obj.num_states,1);
    end
    
    function obj = setNumInputs(obj,num_inputs)
      % Guards the num_inputs variable.
      %  Also pads umin and umax for any new inputs with [-inf,inf].

      if (num_inputs<0) error('num_inputs must be >=0'); end
      obj.num_inputs = num_inputs;
      
      % cut umin and umax to the right size, and pad new inputs with
      % [-inf,inf]
      if (length(obj.umin)~=1 && length(obj.umin)~=num_inputs)
        obj.umin = [obj.umin(1:num_inputs); -inf*ones(max(num_inputs-length(obj.umin),0),1)];
      end
      if (length(obj.umax)~=1 && length(obj.umax)~=num_inputs)
        obj.umax = [obj.umax(1:num_inputs); inf*ones(max(num_inputs-obj.length(obj.umax),0),1)];
      end
      
    end
    
    function obj = setInputLimits(obj,umin,umax)
      % Guards the input limits to make sure it stay consistent
      
      if (length(umin)~=1 && length(umin)~=obj.num_inputs) error('umin is the wrong size'); end
      if (length(umax)~=1 && length(umax)~=obj.num_inputs) error('umax is the wrong size'); end
      if (any(obj.umax<obj.umin)) error('umin must be less than umax'); end
      obj.umin = umin;
      obj.umax = umax;
    end
    
    function u = getDefaultInput(obj)
      % Define the default initial input so that behavior is well-defined
      % if no controller is specified or if no control messages have been
      % received yet.
      u = zeros(obj.num_inputs,1);
    end
    
    function [v dvdy] = stateVectorDiff(obj,X,y)
      % Computes the vector between two points in state space.  
      %   Many algorithms need to this vector, and it's computation can 
      %   be complicated by things like wrapping coordinate systems.  All 
      %   of those details for a given system should be encapsulated in the
      %   stateVectorDiff method.  The default behavior
      %   implemented here simply subtracts the two vectors.
      %
      %   usage:  v = stateVectorDiff(obj,X,y) computes v(:,i) = X(:,i)-y
      %     optional second output dvdy is the gradients of this vector.
      %     Note: to return dvdy, X must be a column vector.
      %

      if (size(X,1)~=obj.num_states) error('X should have num_states rows'); end
      if (size(y,1)~=obj.num_states) error('y should have num_states rows'); end
      if (size(y,2)~=1) error('y can only have a single column'); end
        
      v = X-repmat(y,1,size(X,2));
      if (nargout>1)
        if (size(X,2)>1) error('gradients only for column vector inputs'); end
        dvdy = -eye(length(y));
      end
    end
    
    function d = stateVectorNorm(obj,X,y)
      % Computes the distance between points in state space.  
      %   The default behavior implemented here is the Euclidean distance 
      %   on the stateVectorDiff.

      % note: input error checking with happen immediately in stateVectorDiff
      v = stateVectorDiff(obj,X,y);
      d = sqrt(sum(v.^2,1));
    end
  end

  properties (SetAccess=private, GetAccess = public)
    num_states;  % dimension of x
    num_inputs;  % dimension of u
    umin=-inf;   % constrains u>=umin (default umin=-inf)
    umax=inf;    % constrains u<=uman (default umax=inf)
  end
  
  properties
    ode_solver=@ode45;  % default is @ode45
    ode_options=odeset; % use odeset to change this
  end
  
end