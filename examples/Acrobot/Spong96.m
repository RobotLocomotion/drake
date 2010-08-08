classdef Spong96 < Control

  methods
    function obj = Spong96(acrobot_dynamics)
      obj = obj@Control(4,1);
      
      if (nargin>0)
        typecheck(acrobot_dynamics,'AcrobotDynamics');
        obj.dyn = acrobot_dynamics;

        obj.lqr = AcrobotLQR(acrobot_dynamics);
        obj.control_dt = 0;
      end
    end
    
    function u = control(obj,t,x)
      scope('Acrobot','Energy',1,t,energy(obj,x));
      
      if obj.lqr.isVerified(x)
        u = obj.lqr.control(t,x);
      else
        Etilde = energy(obj,x) - energy(obj,obj.xG);
        scope('Acrobot','Etilde',1,t,Etilde);
        
        theta2ddot_d = - obj.k1*x(2) - obj.k2*x(4) + sat(obj.k3*Etilde*x(3),obj.k3limit);
        scope('Acrobot','theta2ddot_d',1,t,theta2ddot_d);
        u = collocated_pfl(obj,x,theta2ddot_d);
      end
      
        function u = sat(u,lim)
          u = max(min(u,lim),-lim);
        end
    end
    
  
    function [E,T,U] = energy(obj,x)
      c = cos(x(1:2,:)); s = sin(x(1:2,:)); c12 = cos(x(1,:)+x(2,:));
      m1=obj.dyn.m1; m2=obj.dyn.m2; l1=obj.dyn.l1; g=obj.dyn.g; lc1=obj.dyn.lc1; lc2=obj.dyn.lc2; I1=obj.dyn.I1; I2=obj.dyn.I2; b1=obj.dyn.b1; b2=obj.dyn.b2;
      
      T = .5*(I1 + m2*l1^2 + I2 + 2*m2*l1*lc2*c(2,:)).*x(3,:).^2 + .5*I2*x(4,:).^2 + (I2 + m2*l1*lc2*c(2,:)).*x(3,:).*x(4,:);
      U = -m1*g*lc1*c(1,:) - m2*g*(l1*c(1,:) + lc2*c12);
      E = T+U;
    end

    function u = collocated_pfl(obj,x,theta2ddot_d)
      [H,C,B] = manipulatorDynamics(obj.dyn,x(1:2),x(3:4));
      Htilde = H(2,2) - H(2,1)*inv(H(1,1))*H(1,2);
      Ctilde = C(2) - H(2,1)*inv(H(1,1))*C(1);
      u = Htilde*theta2ddot_d + Ctilde;
    end

  end
  
  properties
    dyn;  % AcrobotDynamics class.. used in the design and in the feedback linearization
    lqr;  % The LQR controller at the top
    xG = [pi;0;0;0];  % The goal state
    uG = 0;           % Equilibrium torque at the goal state
    k1 = 10;  % x(2) D gain 
    k2 = 5;   % x(4) D gain
    k3 = 0.3; % Etilde gain
    k3limit = 100;  % Etilde desired q2ddot limit
  end
end