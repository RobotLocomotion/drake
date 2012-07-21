classdef SineSys < DrakeSystem
% Simple example of SOS region of attraction using Taylor approximation
%
% This is a simple follow-on example to the example in xcubed.m 
%
% Consider the system given by
%   xdot = -sin(x)
% Simple inspection reveals that this system has a fixed point at each 
%   x = n*pi for all integers n  (e.g., n=[..,-2,-1,0,1,2,...])
% The fixed points even n are stable, at odd n are unstable.
% 
% In the example below we estimate the region of attraction to the fixed 
% point at x=0 using the toolbox.  Note that we use a Taylor approximation 
% to find a polynomial approximation of the system around the origin. 
%
% The resulting plot demonstrates that the region of attraction estimate is
% tight for the approximate polynomial system, but only an approximation 
% for the original nonlinear system (it is conservative here, but is not 
% guaranteed to be always conservative). 
%
% Note that the same example can be done without approximation using 
% 'Trig-SOS' when we release that code. 
   
  methods
    function obj = SineSys()
      obj = obj@DrakeSystem(1,0,0,1,false,true);
    end
    function xdot = dynamics(obj,t,x,u)
      xdot = -sin(x);
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
  
  methods (Static=true)
    function run()
      
    end
    
    function runTaylor()
      % create a new xcubed object
      p = SineSys();
      
      % taylor expand around the origin (t0=0,x0=0,u0=[]) to order 3.
      pp = taylorApprox(p,0,0,[],3);
      
      % compute region of attraction around x0=0       
      % the levelset V<1 is the region of attraction
      V=regionOfAttraction(pp,0);

      if (V.getFrame ~= pp.getStateFrame) error('oops.  i assumed this was ok'); end
      
      % plot everything.
      xs = -5:.01:5;
      plot(xs,-sin(xs),xs,double(msubs(pp.getPolyDynamics,pp.getStateFrame.poly,xs)),xs,double(msubs(V.getPoly,V.getFrame.poly,xs))-1,xs,0*xs,'linewidth',2);
      axis([-5,5,-1.5,1.5])
      legend('-sin(xs)','poly approx','roa (V-1)');
    end
  end
end
