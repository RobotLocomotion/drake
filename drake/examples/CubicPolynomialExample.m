classdef CubicPolynomialExample < PolynomialSystem
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
    function obj = CubicPolynomialExample()
      obj = obj@PolynomialSystem(1,0,0,1,false,true,false);
    end
    function xdot = dynamicsRHS(obj,t,x,u)
      xdot = -x+x^3;
    end
    function y=output(obj,t,x,u)
      y=x;
    end
    
    function v = constructVisualizer(obj)
      function y=hill(x)
        y = .5*x.^2 - .25*x.^4;
      end
      function draw(t,x)
        xs=linspace(-1.75,1.75,50);
        sfigure(25);
        clf;
        plot(xs,hill(xs),'LineWidth',2,'Color','k');
        hold on;
        r = .15;  % radius of the ball
        th = linspace(-pi,pi,50);
        patch(x+r*sin(th),hill(x)+r+r*cos(th),[.7 .7 1]);
        axis([-1.75,1.75,-.5,.5]);
        axis equal
      end
      
      v = FunctionHandleVisualizer(obj.getOutputFrame,@draw);
    end
  end
  
  methods (Static=true)
    function run()
      % create a new CubicPolynomialExample object
      p = CubicPolynomialExample();

      % compute region of attraction       
      % the levelset V<1 is the region of attraction
      V=regionOfAttraction(p,Point(p.getStateFrame,0));
      
      % display the polynomial representation of V that results
      display(V.getPoly);
    end
    
    function sosExample()
      checkDependency('spotless');
      checkDependency('mosek');
      
      % Define a new symbolic variable
      x=msspoly('x');
      
      % Define the dynamics and Lyapunov function
      f = -x + x^3;
      V = x^2;
      Vdot = diff(V,x)*f;
      
      % construct a SOS program
      prog=spotsosprog;
      prog = prog.withIndeterminate(x);
      
      % construct the langrange multiplier
      m=monomials(x,0:4);
      [prog,c] = prog.newFree(length(m));
      lambda = c'*m;
      prog = prog.withSOS(lambda);
      % note: this could all be done with a single line:
      % [prog,lambda] = prog.newSOSPoly(monomials(x,0:4));
      
      % construct the slack variables (which must be positive)
      [prog,slack] = prog.newPos(1);
      
      % add the SOS constraints
      prog = prog.withSOS(-(Vdot+lambda*(1-V))-slack*x^2);
      
      % solve the problem using -slack as the objective
      options = spot_sdp_default_options();
      options.verbose = 0;
      sol = prog.minimize(-slack,@spot_mosek,options);
      
      % display the results
      slack = double(sol.eval(slack))
      c = double(sol.eval(c))
      
      
      %% Everything after this is just plotting...
      
      xs=-1.2:.05:1.2;
      clear lambda vl;
      for i=1:length(xs)
        lambda(i)=double(subs(c'*m,x,xs(i)));
        vl(i)=double(subs(-2*x^2+2*x^4+(c'*m)*(1-x^2),x,xs(i)));
      end
      
      clf;
      hold on;
      plot(xs,-xs+xs.^3,'k','linewidth',2);
      legend('xdot');
      axis tight
%      pause;
      
      plot(xs,-2*xs.^2+2*xs.^4,'r','linewidth',2);
      legend('xdot','Vdot');
%      pause;
      
      plot(xs,lambda,'g','linewidth',2);
      legend('xdot','Vdot','lambda');
%      pause;
      
      plot(xs,vl,'b','linewidth',2);
      legend('xdot','Vdot','lambda','Vdot+lambda*(1-V)');
      
    end
    
    function animate()
      p=CubicPolynomialExample();
      v=p.constructVisualizer();

      x1=p.simulate([0 5],.8);
      x2=p.simulate([0 1.5],1.02);
      x2=x2.shiftTime(5);
      x3=p.simulate([0 5],-.9);
      x3=x3.shiftTime(6.5);
      x=HybridTrajectory({x1,x2,x3});
      v.playback(x);
%      v.playbackSWF(x,'CubicPolynomialExample');
    end
    
    function findFixedPointTest()
      p=CubicPolynomialExample();
      
      xstar = findFixedPoint(p,.1*randn,[]);
      valuecheck(xstar,0,1e-4);

      xstar = findFixedPoint(p,1+.1*randn,[]);
      valuecheck(xstar,1,1e-4);
      
      xstar = findFixedPoint(p,-1+.1*randn,[]);
      valuecheck(xstar,-1,1e-4);
    end
  end
end
