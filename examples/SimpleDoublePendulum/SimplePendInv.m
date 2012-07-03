classdef SimplePendInv < DrakeSystem
  % demonstration of feedback linearization (used in lecture 1 of 6.832)
  % Double Pendulum is made "feedback equivalent" to a Simple Pendulum
  
  properties 
    p
  end
  
  methods
    function obj = SimplePendInv(plant)
      obj = obj@DrakeSystem(0,0,4,2,true,true);
      obj.p = plant;
      obj = obj.setInputFrame(plant.getStateFrame);
      obj = obj.setOutputFrame(plant.getInputFrame);
    end
    
    function u = output(obj,t,junk,x)
      [H,C,B] = obj.p.manipulatorDynamics(x(1:2),x(3:4));
      l = obj.p.l1+obj.p.l2;
      b = .1;
      u = C + H*[obj.p.g*sin(x(1))/l - b*x(3);0];
    end
  end 
  
end
