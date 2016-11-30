classdef SecondOrderSystem < DrakeSystem
% An abstract class that wraps qddot = f(t,q,qdot,u).
%   A specialization of the Dynamics class for systems of second order.  
  
  methods
    function obj = SecondOrderSystem(num_q,num_u,timeInvariantFlag)
    % SecondOrderPlant(num_q, num_u)
    %   Constructs a SecondOrderPlant object with num_q configuration
    %   variables (implying num_q*2 states) and num_u control inputs.

%      if (nargin>0)
        obj = obj@DrakeSystem(num_q*2,0,num_u,num_q*2,false,timeInvariantFlag);
        obj = obj.setNumDOF(num_q);
%      end
    end
  end
  
  methods (Abstract=true)
    % implements qdd = f(t,q,qd,u)
    qdd = sodynamics(obj,t,q,qd,u)
  end

  methods % (Sealed=true) % todo: re-enable this (requires updating generated gradients in acrobot, cartpole, etc classes.)
    function [xdot,varargout] = dynamics(obj,t,x,u)
    % Provides the dynamics interface for sodynamics
      q=x(1:obj.num_q); qd=x((obj.num_q+1):end);
      if (nargout>1)
        varargout = cell(1,nargout-1);
        [qdd,varargout{:}] = obj.sodynamics(t,q,qd,u);
        varargout{1} = [zeros(obj.num_q,1+obj.num_q), eye(obj.num_q), zeros(obj.num_q,obj.num_u); varargout{1}];
        for i=2:(nargout-1)
          varargout{i} = [zeros(obj.num_q,1+obj.num_q+obj.num_u);varargout{i}];
        end
      else
        qdd = obj.sodynamics(t,q,qd,u);
      end
      xdot = [qd;qdd];
    end
  end
  
  methods
    function obj = setNumDOF(obj,num_q)
    % Guards the num_q variable to make sure it stays consistent 
    % with num_x.
      obj = setNumContStates(obj,num_q*2);
    end

    function num_q = getNumDOF(obj)
      % Return the number of degrees of freedom in the system
      num_q = obj.num_q;
    end
    
    function obj = setNumContStates(obj,num_xc)
    % Guards the num_xc variable to make sure it stays consistent
    % with num_q
      if (num_xc<0 || rem(num_xc,2)) error('num_x must be even for a SecondOrderDynamics'); end 
      obj.num_q = num_xc/2;
      obj = setNumContStates@DrakeSystem(obj,num_xc);
    end
    
    
    function y = output(obj,t,x,u)
      % default output is the full state
      y = x;
    end
        
    
    function sys = feedback(sys1,sys2)
      % Attempt to produce a new second-order system if possible
      if (isa(sys2,'SecondOrderSystem'))
        % todo: implement this
        warning('feedback combinations of second order systems not handled explicitly yet. kicking out to a combination of DrakeSystems');
      end
      sys = feedback@DrakeSystem(sys1,sys2);
    end
    
    function sys = cascade(sys1,sys2)
      % Attempt to produce a new second-order system if possible
      if (isa(sys2,'SecondOrderSystem'))
        % todo: implement this
        warning('cascade combinations of second order systems not handled explicitly yet. kicking out to a combination of DrakeSystems');
      end
      sys = cascade@DrakeSystem(sys1,sys2);
    end
    
    function varargout = pdcontrol(sys,Kp,Kd,index)
      % creates new blocks to implement a PD controller, roughly
      % illustrated by
      %   q_d --->[ Kp ]-->(+)----->[ sys ]----------> [q;qd]
      %                     | -                 |
      %                     -------[ Kp,Kd ]<---- 
      %                       
      % when invoked with a single output argument:
      %   newsys = pdcontrol(sys,...)
      % then it returns a new system which contains the new closed loop
      % system containing the PD controller and the plant.
      %
      %  u = Kp*(q_d - q) - Kd*qd
      %
      % when invoked with two output arguments:
      %   [pdff,pdfb] = pdcontrol(sys,...)
      % then it return the systems which define the feed-forward path and
      % feedback-path of the PD controller (but not the closed loop
      % system).
      %
      % @param Kp a num_u x num_u matrix with the position gains
      % @param Kd a num_u x num_u matrix with the velocity gains
      % @param index a num_u dimensional vector specifying the mapping from q to u.
      % index(i) = j indicates that u(i) actuates q(j). @default: 1:num_u
      %
      % For example, the a 2D floating base (with adds 3 passive joints in 
      % positions 1:3)model with four actuated joints might have 
      %      Kp = diag([10,10,10,10])
      %      Kd = diag([1, 1, 1, 1])
      %      index = 4:7
      %   newsys = pdcontrol(sys,Kp,Kd,index);
      
      % todo: consider adding an option that would give [q_d;qd_d] as in
      % input.  would be trivial to type in, but doesn't add any richness
      % to the control input (and increases the dimensionality)
      
      % todo: consider allowing the user to specify less than num_u gains
      % (e.g., PD control just part of the system).  And pass the original
      % input through on the other inputs.
      
      sizecheck(Kp,[sys.num_u,sys.num_u]);
      sizecheck(Kd,[sys.num_u,sys.num_u]);
      
      if nargin<4 || isempty(index)
        index = 1:sys.num_u;
      end
      sizecheck(index,sys.num_u);
      rangecheck(index,0,sys.num_q);
      
      % pdfb = prop-derivative control feedback term:
      % tau = -Kp*theta - Kd*thetadot
      D = zeros(sys.num_u,sys.num_x);
      D(:,index) = -Kp;
      D(:,sys.num_q + index) = -Kd;
      pdfb = LinearSystem([],[],[],[],[],D);
      pdfb = setOutputFrame(pdfb,sys.getInputFrame);
      pdfb = setInputFrame(pdfb,sys.getStateFrame);  % note: assume full-state feedback for now
      
      % pdff = prop-derivative control feedforward term:
      % tau = Kp*thetadesired
      pdff = LinearSystem([],[],[],[],[],Kp*eye(sys.num_u));
      pdff = setOutputFrame(pdff,sys.getInputFrame);
      coordinates = sys.getStateFrame.getCoordinateNames();
      pdff = setInputFrame(pdff,CoordinateFrame('q_d',length(index),'d',{coordinates{index}}));

      if nargout>1
        varargout{1} = pdff;
        varargout{2} = pdfb;
      else
%        sys = cascade(cascade(ScopeSystem(sys.getInputFrame),sys),ScopeSystem(sys.getInputFrame));
        varargout{1} = cascade(pdff,feedback(sys,pdfb));
      end
    end
    
  end
  
  properties (SetAccess = private, GetAccess = public)
    num_q  % number of configuration variables
  end
  
  
end
