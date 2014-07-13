classdef XCubed < PolynomialSystem
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
% an LyapunovFunction object equivalent to V=x^2.  The region of attraction is the
% one-sub-level set {x : V(x)<1} of the this polynomial, or x=(-1,1) 
  
  methods
    function obj = XCubed()
      obj = obj@PolynomialSystem(1,0,0,1,false,true,false);
    end
    function xdot = dynamicsRHS(obj,t,x,u)
      xdot = -x+x^3;
    end
    function y=output(obj,t,x,u)
      y=x;
    end
  end
  
  methods (Static=true)
    function run()
      % create a new xcubed object
      p = XCubed();

      % compute region of attraction       
      % the levelset V<1 is the region of attraction
      V=regionOfAttraction(p,Point(p.getStateFrame,0));
      display(V.getPoly);
    end
    
    function animate()
      p=XCubed();
      v=XCubedVisualizer(p);

      x1=p.simulate([0 5],.8);
      x2=p.simulate([0 1.5],1.02);
      x2=x2.shiftTime(5);
      x3=p.simulate([0 5],-.9);
      x3=x3.shiftTime(6.5);
      x=HybridTrajectory({x1,x2,x3});
      v.playback(x);
%      v.playbackSWF(x,'xcubed');
    end
    
    function findFixedPointTest()
      p=XCubed();
      
      xstar = findFixedPoint(p,.1*randn,[]);
      valuecheck(xstar,0,1e-4);

      xstar = findFixedPoint(p,1+.1*randn,[]);
      valuecheck(xstar,1,1e-4);
      
      xstar = findFixedPoint(p,-1+.1*randn,[]);
      valuecheck(xstar,-1,1e-4);
    end
  end
end
