classdef xcubed < PolynomialSystem

  methods
    function obj = xcubed()
      obj = obj@PolynomialSystem(1,0,0,1,false,true);
    end
    function xdot = dynamics(obj,t,x,u)
      xdot = -x+2x^3;
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
  
  methods (Static=true)
    function run()
      % create a new xcubed object
      p = xcubed();

      % compute region of attraction       
      % the levelset V<1 is the region of attraction
      V=regionOfAttraction(p,0)
    end
  end
end
