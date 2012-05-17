classdef ManipulatorPlant < SecondOrderPlant
% An abstract class that wraps H(q)qddot + C(q,qdot,f_ext) = B(q)u.
% Coming soon:  will also support bilateral constraints of the form: phi(q)=0.

  methods
    function obj = ManipulatorPlant(num_q, num_u, num_bilateral_constraints)
      obj = obj@SecondOrderPlant(num_q,num_u,true);
      if (nargin>2)  % else num_bilateral_constraints=0 by default
        obj = obj.setNumBilateralConstraints(num_bilateral_constraints);
      end
    end
  end
  
  methods (Abstract=true)
    %  H(q)qddot + C(q,qdot,f_ext) = Bu
    [H,C,B] = manipulatorDynamics(obj,q,qd);
  end

  methods
    function x0 = getInitialState(obj)
      valid=false;
      count=0;
      while (~valid)
        if (count>=10)
          error('tried 10 random initial conditions and still cannot resolve the constraints.');
        end
        x0 = randn(obj.num_xd+obj.num_xc,1);
        [x0,valid] = resolveConstraints(obj,x0(1:obj.num_q));
        count=count+1;
      end
    end

    function qdd = sodynamics(obj,t,q,qd,u)
    % Provides the SecondOrderDynamics interface to the manipulatorDynamics.
      [H,C,B] = manipulatorDynamics(obj,q,qd);
      
      if (obj.num_u>0) tau=B*u; else tau=zeros(obj.num_q,1); end
      if (obj.num_bilateral_constraints)
        [phi,J,dJ] = geval(@obj.bilateralConstraints,q);

%        phi
        tol = 1e-6;
        % todo: make this a more proper tolerance (which can be set
        % independently, or which is derived from the ode parameters?)
%        if (any(abs(phi)>tol)) error('this state violates the bilateral constraints');  end
        
        Hinv = inv(H);

        Jdotqd = dJ*reshape(qd*qd',obj.num_q^2,1);
%        k_constraint=1;
%        kd_constraint=.1;
        constraint_force = J'*(inv(J*Hinv*J')*(J*Hinv*C - Jdotqd));% - k_constraint*phi - kd_constraint*J*qd);
        qdd = Hinv*(tau + constraint_force - C);
      else
        qdd = H\(tau - C);
      end
      
    end
    
    function obj = setNumBilateralConstraints(obj,num_bilateral_constraints)
    % Set the number of bilateral constraints
      if (~isscalar(num_bilateral_constraints) || num_bilateral_constraints <0)
        error('num_bilateral_constraints must be a non-negative scalar');
      end
      obj.num_bilateral_constraints=num_bilateral_constraints;
      obj = setNumStateConstraints(obj,num_bilateral_constraints);
    end
    
    function phi = bilateralConstraints(obj,q)
      error('manipulators with constraints must implement the bilateralConstraints method');
    end
    
    function phi = stateConstraints(obj,x)
      phi = bilateralConstraints(obj,x(1:obj.num_q));
    end
    
  end
  
  
  properties (SetAccess = private, GetAccess = public)
    num_bilateral_constraints = 0  % the number of bilateral constraints of the form phi(q)=0
  end
end
