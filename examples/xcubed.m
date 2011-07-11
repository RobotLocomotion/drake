classdef xcubed < PolynomialSystem
% Simple SOS region of attraction example.
%
% Consider the first-order, one-dimensional, dynamical system governed by
%  xdot = -x + x^3
% Simple graphical analysis (e.g., as seen in Steve Strogatz'book on
% Nonlinear Dynamics) reveals that this system has 3 fixed points: 
%   x = -1, 0, and 1.  
% Of these, only the fixed point at x=0 is stable, and it's region of
% attraction is clearly the open set x=(-1,1). 
%
% In this example, we obtain the same result using the toolbox.  By
% constructing a simple PolynomialSystem object and populating the dynamics
% method, we can simply call "regionOfAttraction" which should return
% an msspoly object equivalent to V=x^2.  The region of attraction is the
% one-sub-level set {x : V(x)<1} of the this polynomial, or x=(-1,1) 
  
  methods
    function obj = xcubed()
      obj = obj@PolynomialSystem(1,0,0,1,false,true);
    end
    function xdot = dynamics(obj,t,x,u)
      xdot = -x+x^3;
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
