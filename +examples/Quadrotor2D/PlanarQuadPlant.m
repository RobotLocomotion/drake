classdef PlanarQuadPlant < SecondOrderSystem

  % state:  
  %  q(1) - x position
  %  q(2) - z position
  %  q(3) - pitch (theta)
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    L = 0.25; % length of rotor arm
    m = 0.486; % mass of quadrotor
    I = 0.00383; % moment of inertia
    g = 9.81; % gravity
  end
  
  methods
    function obj = PlanarQuadPlant()
      obj = obj@SecondOrderSystem(3,2,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      qdd = [ -sin(q(3))/obj.m*(u(1)+u(2));
        -obj.g + cos(q(3))/obj.m*(u(1)+u(2));
        obj.L/obj.I*(-u(1)+u(2))];
    end
    
    function x = getInitialState(obj)
      x = randn(6,1);
    end
    
    function [c,V] = hoverLQR(obj)
      x0 = Point(obj.getStateFrame,zeros(6,1));
      u0 = Point(obj.getInputFrame,obj.m*obj.g/2 * [1;1]);
      Q = diag([10 10 10 1 1 (obj.L/2/pi)]);  %Q = diag([10*ones(1,3) ones(1,3)]);
      R = [0.1 0.05; 0.05 0.1];  %R = diag([0.1 0.1]);

      if (nargout>1)
        [c,V0] = tilqr(obj,x0,u0,Q,R);
        sys = feedback(obj,c);

        pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
        options=struct();
        options.degL1=2;
        %options.method='bilinear';
        %options.degV=4;
        V=regionOfAttraction(pp,V0,options);
      else
        c = tilqr(obj,x0,u0,Q,R);
      end
    end
    
  end
  
end
