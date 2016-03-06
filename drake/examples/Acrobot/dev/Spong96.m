classdef Spong96 < MemorylessControl
  % Implements the swing up control from Spong96.  Doesn't work well.
  %
  % Consider running the lcmScope in another matlab instance for additional
  % debugging info.
  %
  % Unless there is something that I don't understand, the swingup
  % controller does NOT work very well at all.  The energy pumping and the
  % regulating q2 terms clash badly, making it extremely hard to tune, and
  % very hard to verify.  If the q2 terms are off, then Etilde goes to zero
  % nicely, but the second link spins out of control.  This has completely
  % shattered my impression of the controller - it now feels like a
  % terrible hack that is very difficult to make work (I hope that I just
  % have a bug).  - Russ

  methods
    function obj = Spong96(plant)
      obj = obj@MemorylessControl(4,1);
      
      if (nargin>0)
        typecheck(plant,'AcrobotPlant');
        obj.plant = plant;

        obj.lqr = AcrobotLQR(plant);
%        obj.control_dt = 0;
      end
    end
    
    function u = control(obj,t,x)
      if obj.lqr.isVerified(x)
        u = obj.lqr.control(t,x);
      else
        Etilde = energy(obj,x) - energy(obj,obj.xG);
        scope('Acrobot','Etilde',t,Etilde,struct('linespec','r','scope_id',2));
        
        q2wrapped = mod(x(2)+pi,2*pi)-pi;
        q2dd_d = - obj.k1*q2wrapped - obj.k2*x(4) + obj.k3*sat(Etilde*x(3),obj.k3limit);
        u = collocated_pfl(obj,x,q2dd_d);
      end
      
      scope('Acrobot','tau',t,u);
      scope('Acrobot','lqr',t,10*double(obj.lqr.isVerified(x)),struct('linespec','r'));
      
        function u = sat(u,lim)
          u = max(min(u,lim),-lim);
        end
    end
    
  
    function [E,T,U] = energy(obj,x)
      c = cos(x(1:2,:)); s = sin(x(1:2,:)); c12 = cos(x(1,:)+x(2,:));
      m1=obj.plant.m1; m2=obj.plant.m2; l1=obj.plant.l1; g=obj.plant.g; lc1=obj.plant.lc1; lc2=obj.plant.lc2; I1=obj.plant.I1; I2=obj.plant.I2; b1=obj.plant.b1; b2=obj.plant.b2;
      
      [H,phi,B] = manipulatorDynamics(obj.plant,x(1:2),x(3:4));
      T = .5*x(3:4)'*H*x(3:4);
      %      T = .5*(I1 + m2*l1^2 + I2 + 2*m2*l1*lc2*c(2,:)).*x(3,:).^2 + .5*I2*x(4,:).^2 + (I2 + m2*l1*lc2*c(2,:)).*x(3,:).*x(4,:);
      U = -m1*g*lc1*c(1,:) - m2*g*(l1*c(1,:) + lc2*c12);
      E = T+U;
    end

    function u = collocated_pfl(obj,x,q2dd_d)
      [H,phi,B] = manipulatorDynamics(obj.plant,x(1:2),x(3:4));
      Htilde = H(2,2) - H(2,1)*inv(H(1,1))*H(1,2);
      phitilde = phi(2) - H(2,1)*inv(H(1,1))*phi(1);
      u = Htilde*q2dd_d + phitilde;
    end

  end
  
  properties
    plant;  % AcrobotPlant class.. used in the design and in the feedback linearization
    lqr;  % The LQR controller at the top
    xG = [pi;0;0;0];  % The goal state
    uG = 0;           % Equilibrium torque at the goal state
    k1 = 6;  % q(2) P gain 
    k2 = 2;   % qd(2) D gain
    k3 = 1.5; % Etilde gain
    k3limit = 40;  % Etilde desired q2ddot limit
  end
end