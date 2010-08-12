classdef EnergyShaping < Control
  
  methods
    function obj = EnergyShaping(acrobot_dynamics)
      obj = obj@Control(4,1);
      
      if (nargin>0)
        typecheck(acrobot_dynamics,'AcrobotDynamics');
        obj.dyn = acrobot_dynamics;

        obj.lqr = AcrobotLQR(acrobot_dynamics);
        obj.control_dt = 0;
      end
    end
    
    function u = control(obj,t,x)
      if obj.lqr.isVerified(x)
        u = obj.lqr.control(t,x);
      else
        Etilde = energy(obj,x) - energy(obj,obj.xG);
        scope('Acrobot','Etilde',t,Etilde,struct('scope_id',2,'linespec','r'));

        q2wrapped = mod(x(2)+pi,2*pi)-pi;
        u = -obj.k1*q2wrapped - obj.k2*x(4) -obj.k3*x(4)*Etilde;
      end
      
      scope('Acrobot','tau',t,u);
      scope('Acrobot','lqr',t,10*double(obj.lqr.isVerified(x)),struct('linespec','r'));
      
    end
    
    function [E,T,U] = energy(obj,x)
      c = cos(x(1:2,:)); s = sin(x(1:2,:)); c12 = cos(x(1,:)+x(2,:));
      m1=obj.dyn.m1; m2=obj.dyn.m2; l1=obj.dyn.l1; g=obj.dyn.g; lc1=obj.dyn.lc1; lc2=obj.dyn.lc2; I1=obj.dyn.I1; I2=obj.dyn.I2; b1=obj.dyn.b1; b2=obj.dyn.b2;
      
      [H,phi,B] = manipulatorDynamics(obj.dyn,x(1:2),x(3:4));
      T = .5*x(3:4)'*H*x(3:4);
      %      T = .5*(I1 + m2*l1^2 + I2 + 2*m2*l1*lc2*c(2,:)).*x(3,:).^2 + .5*I2*x(4,:).^2 + (I2 + m2*l1*lc2*c(2,:)).*x(3,:).*x(4,:);
      U = -m1*g*lc1*c(1,:) - m2*g*(l1*c(1,:) + lc2*c12);
      E = T+U;
    end
    
  end
  
  properties
    dyn;  % AcrobotDynamics class.. used in the design and in the feedback linearization
    lqr;  % The LQR controller at the top
    xG = [pi;0;0;0];  % The goal state
    uG = 0;           % Equilibrium torque at the goal state
    k1 = 2;  % q(2) P gain
    k2 = 1;   % qd(2) D gain
    k3 = .1; % Etilde gain
 end
  
end
