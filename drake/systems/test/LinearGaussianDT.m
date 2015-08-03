classdef LinearGaussianDT < StochasticDrakeSystem

 methods 
   function obj = LinearGaussianDT
     obj = obj@StochasticDrakeSystem(...
       0, ... % number of continuous states
       1, ... % number of discrete states
       0, ... % number of inputs
       1, ... % number of outputs
       false, ...  % not direct feedthrough
       true, ...   % time invariant
       1, ... % number of noise inputs
       .1);  % time constant of w(t)
     
     obj = setSampleTime(obj,[.1;0]);
   end
   
   function xdn = stochasticUpdate(obj,t,x,u,w)
     xdn = w;
     
%     if (t-floor(t)>0) error('this should only be called at the sample times'); end
   end
   
   function x0 = getInitialState(obj)
     x0 = randn;
   end
   
   function y = stochasticOutput(obj,t,x,u,w);
     y=x;
   end
 end

 methods (Static)
   function run
     sys=LinearGaussianDT;
     xtraj = simulate(sys,[0 1000]);
     fnplt(xtraj);
     
     % check that I'm actually getting variance 1 noise
     valuecheck(1,var(eval(xtraj,getBreaks(xtraj))),.1);  
   end
 end
 
end

