classdef Spong96 < Control
  % technically not the same controller as Spong96.  He goes through PFL.
  % I'm just doing energy shaping on the non-PFL'd system here.
  
  properties
    lqr
    plant
    k1 = 10; k2 = 5; k3 = 10; k3limit = 50;
  end
  
  methods
    function obj = Spong96(plant)
      obj = obj@Control(4,1);
      if (nargin>0)
        typecheck(plant,'CartPolePlant')
        obj.plant = plant;
        obj.lqr = CartPoleLQR(plant);
      end
    end
    
    function u = control(obj,t,x)
      if (isVerified(obj.lqr,x))
        u = control(obj.lqr,t,x);
      else
        Etilde = pend_energy(obj,x) - 1.05*pend_energy(obj,obj.lqr.x0);
        u = -obj.k1*x(1) - obj.k2*x(3) + sat(obj.k3*Etilde*cos(x(2))*x(4),obj.k3limit);
        scope('CartPole','phase',x(2),x(4),struct('resetOnXval',false));
      end

      % plot some useful quantities
      [bverified,vt,conf] = isVerified(obj.lqr,x);
      scope('CartPole','zero',t,0,struct('scope_id',2,'linespec','r'));
      scope('CartPole','conf',t,conf,struct('scope_id',2,'linespec','b'));
      
        function u = sat(u,lim)
          u = max(min(u,lim),-lim);
        end
    end
    
    function E = pend_energy(obj,x)
      E = .5*obj.plant.mp*obj.plant.l^2*x(4,:).^2 - obj.plant.mp*obj.plant.g*obj.plant.l*cos(x(2,:));
    end
    
  end
  
end