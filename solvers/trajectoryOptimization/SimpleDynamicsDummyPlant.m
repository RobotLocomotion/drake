classdef SimpleDynamicsDummyPlant < SecondOrderSystem
  methods
    function obj = SimpleDynamicsDummyPlant(nq)
      obj = obj@SecondOrderSystem(nq,0,true);
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      qdd = zeros(obj.num_q,1);
    end
  end
end