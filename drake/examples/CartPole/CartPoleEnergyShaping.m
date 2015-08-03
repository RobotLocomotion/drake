classdef CartPoleEnergyShaping < DrakeSystem
  % technically not the same controller as Spong96.  He goes through PFL.
  % I'm just doing energy shaping on the non-PFL'd system here.
  
  properties
    p  % plant
    k1 = 40; k2 = 15; k3 = 15; k3limit = 50;
  end
  
  methods
    function obj = CartPoleEnergyShaping(plant)
      obj = obj@DrakeSystem(0,0,4,1,true,true);
      if (nargin>0)
        typecheck(plant,'CartPolePlant');
        obj.p = plant;
        obj = obj.setInputFrame(plant.getStateFrame);
        obj = obj.setOutputFrame(plant.getInputFrame);
      end
    end
    
    function u = output(obj,t,~,x)
      Etilde = pend_energy(obj,x) - 1.05*pend_energy(obj,[0;pi;0;0]);
      u = -obj.k1*x(1) - obj.k2*x(3) + sat(obj.k3*Etilde*cos(x(2))*x(4),obj.k3limit);
      scope('CartPole','phase',x(2),x(4),struct('resetOnXval',false));

      function u = sat(u,lim)
        u = max(min(u,lim),-lim);
      end
    end
    
    function E = pend_energy(obj,x)
      E = .5*obj.p.mp*obj.p.l^2*x(4,:).^2 - obj.p.mp*obj.p.g*obj.p.l*cos(x(2,:));
    end
  end
  
  methods (Static)
    function run()
      cp = CartPolePlant();
      c = CartPoleEnergyShaping(cp);
      v = CartPoleVisualizer(cp);
      sys = feedback(cp,c);

      for i=1:5
        x=simulate(sys,[0 5]);
        playback(v,x);
      end
    end
  end
end
