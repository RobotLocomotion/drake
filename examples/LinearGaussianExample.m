classdef LinearGaussianExample < StochasticRobotLibSystem

 methods 
   function obj = LinearGaussianExample
     obj = obj@StochasticRobotLibSystem(1,0,1,1,false,true,1,.1);
   end
   
   function xcdot = dynamics(obj,t,x,u,w)
     xcdot = -x + w;
   end
   
   function x0 = getInitialState(obj)
     x0 = randn;
   end
   
   function y = output(obj,t,x,u,w);
     y=x;
   end
 end

 methods (Static)
   function run
     fnplt(simulate(LinearGaussianExample,[0 5]));
   end
 end
 
end

